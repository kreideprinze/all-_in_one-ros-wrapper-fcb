#include "fcb_camera/visca_link.hpp"

#include "fcb_camera/board.hpp"

#include <fcntl.h>
#include <glob.h>
#include <poll.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <thread>

namespace fcb_camera
{
namespace
{
speed_t baudConstant(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: throw std::runtime_error("unsupported VISCA baud rate " + std::to_string(baud));
  }
}

// Broadcast Address Set and its answer; see the header.
const visca::Packet kPad = {0x88, 0x30, 0x01, 0xFF};
const visca::Packet kPadReply = {0x88, 0x30, 0x02, 0xFF};
// Eight answers are 32 bytes: a full USB packet behind the awaited reply,
// however much of the previous packet the board was already holding.
constexpr int kPadsPerBurst = 8;
// A quiet line is padded again only after this long since the last pads.
// Eight pads took 120-180 ms to come back on the drone; padding faster than
// that flooded the camera and the board went silent altogether.
constexpr auto kPadAgain = std::chrono::milliseconds(400);
// A padded answer takes 120-250 ms, now and then 500.
constexpr auto kPaddedMinTimeout = std::chrono::milliseconds(1000);

std::vector<std::string> globPaths(const char * pattern)
{
  std::vector<std::string> out;
  glob_t g{};
  if (::glob(pattern, 0, nullptr, &g) == 0) {
    for (size_t i = 0; i < g.gl_pathc; ++i) out.emplace_back(g.gl_pathv[i]);
  }
  globfree(&g);
  return out;
}
}  // namespace

ViscaLink::ViscaLink(const std::string & port, int baud, uint8_t address, Duration timeout, int pad_replies)
: port_(port), baud_(baud), address_(address), timeout_(timeout)
{
  pad_ = pad_replies < 0 ? boardFor(port).pads_visca_replies : pad_replies != 0;
  if (pad_) {
    timeout_ = std::max(timeout_, kPaddedMinTimeout);
    desynced_ = true;  // the board may still hold bytes from the port's last owner
  }
  const speed_t speed = baudConstant(baud);
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::runtime_error("cannot open " + port + ": " + std::strerror(errno));
  }
  if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error(port + " is locked by another process; refusing to share the VISCA bus");
  }
  termios tio{};
  if (::tcgetattr(fd_, &tio) != 0) {
    const std::string err = std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("tcgetattr " + port + ": " + err);
  }
  cfmakeraw(&tio);
  tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
  tio.c_cflag |= CS8 | CLOCAL | CREAD;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
    const std::string err = std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("tcsetattr " + port + ": " + err);
  }
  ::tcflush(fd_, TCIOFLUSH);
}

ViscaLink::~ViscaLink()
{
  if (fd_ >= 0) {
    ::flock(fd_, LOCK_UN);
    ::close(fd_);
  }
}

void ViscaLink::flushInput() { ::tcflush(fd_, TCIFLUSH); }

void ViscaLink::pad()
{
  Packet burst;
  for (int i = 0; i < kPadsPerBurst; ++i) burst.insert(burst.end(), kPad.begin(), kPad.end());
  write(burst);
  paddedAt_ = std::chrono::steady_clock::now();
}

void ViscaLink::send(const Packet & frame)
{
  if (desynced_) resync();
  write(frame);
  if (pad_) pad();
}

void ViscaLink::resync()
{
  // Sixteen pads out, eight answers back: everything ahead of those answers
  // is stale, and what stays held in the board afterwards is only pad answers.
  pad();
  pad();
  const auto deadline = std::chrono::steady_clock::now() + std::max(timeout_, kPaddedMinTimeout);
  int seen = 0;
  while (seen < kPadsPerBurst) {
    if (readFrame(deadline) == kPadReply) ++seen;
  }
  desynced_ = false;
}

void ViscaLink::write(const Packet & frame)
{
  if (pad_) {
    // Paced to the board's own serial rate, one VISCA packet at a time. The
    // USB side takes a whole burst at once; a padded batch is ~100 bytes, far
    // more than the board ever got from the single-question Python recorder,
    // and while it was also streaming video the board started throwing USB
    // errors (-71) and dropped off the bus. Holding back to the 9600-baud
    // line keeps at most one packet waiting in it.
    size_t start = 0;
    for (size_t i = 0; i < frame.size(); ++i) {
      if (frame[i] != 0xFF && i + 1 != frame.size()) continue;
      writeRaw(Packet(frame.begin() + start, frame.begin() + i + 1));
      std::this_thread::sleep_for(std::chrono::microseconds((i + 1 - start) * 10'000'000LL / baud_));
      start = i + 1;
    }
    return;
  }
  writeRaw(frame);
}

void ViscaLink::writeRaw(const Packet & frame)
{
  size_t off = 0;
  while (off < frame.size()) {
    const ssize_t n = ::write(fd_, frame.data() + off, frame.size() - off);
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) {
        pollfd p{fd_, POLLOUT, 0};
        ::poll(&p, 1, 100);
        continue;
      }
      throw std::runtime_error(std::string("VISCA write failed: ") + std::strerror(errno));
    }
    off += static_cast<size_t>(n);
  }
  ::tcdrain(fd_);
}

ViscaLink::Packet ViscaLink::readFrame(std::chrono::steady_clock::time_point deadline)
{
  Packet frame;
  for (;;) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      throw ViscaTimeout("no complete VISCA frame within timeout (partial: " + visca::hex(frame) + ")");
    }
    int ms = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
    // On a padding link wake up in time to pad again rather than sleeping
    // through the rest of the exchange.
    if (pad_) ms = std::min(ms, 25);
    pollfd p{fd_, POLLIN, 0};
    const int rc = ::poll(&p, 1, std::max(1, ms));
    if (rc < 0) {
      if (errno == EINTR) continue;
      throw std::runtime_error(std::string("VISCA poll failed: ") + std::strerror(errno));
    }
    if (rc == 0) {
      // Timed per exchange, not per frame: re-padding every time a frame's
      // worth of pad answers trickled in is what flooded the camera.
      if (pad_ && std::chrono::steady_clock::now() - paddedAt_ >= kPadAgain) pad();
      continue;
    }
    uint8_t byte;
    const ssize_t n = ::read(fd_, &byte, 1);
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) continue;
      throw std::runtime_error(std::string("VISCA read failed: ") + std::strerror(errno));
    }
    if (n == 0) continue;
    frame.push_back(byte);
    if (byte == 0xFF) return frame;
    if (frame.size() > 64) frame.clear();  // garbage without terminator; resync
  }
}

ViscaLink::Packet ViscaLink::await(visca::ReplyKind want, Duration timeout, size_t min_payload)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  for (;;) {
    Packet frame;
    try {
      frame = readFrame(deadline);
    } catch (const ViscaTimeout &) {
      // Its reply may yet surface behind the board's packet boundary; clear it
      // out before the next exchange rather than read it as that one's answer.
      if (pad_) desynced_ = true;
      throw;
    }
    const visca::Reply reply = visca::classify(frame);
    if (reply.kind == visca::ReplyKind::Error) {
      throw ViscaError(visca::errorMessage(reply.payload));
    }
    if (reply.kind == want && reply.payload.size() >= min_payload) {
      return reply.payload;
    }
    // An ACK we are not waiting on, or a late completion of an earlier
    // movement: drop it and keep reading.
  }
}

void ViscaLink::command(const Packet & frame, bool wait_completion, Duration timeout)
{
  if (timeout.count() <= 0) timeout = timeout_;
  std::lock_guard<std::mutex> lock(mutex_);
  send(frame);
  await(visca::ReplyKind::Ack, timeout_, 1);
  if (wait_completion) await(visca::ReplyKind::Completion, timeout, 1);
}

void ViscaLink::commandNoAck(const Packet & frame, Duration timeout)
{
  if (timeout.count() <= 0) timeout = timeout_;
  std::lock_guard<std::mutex> lock(mutex_);
  send(frame);
  await(visca::ReplyKind::Completion, timeout, 1);
}

ViscaLink::Packet ViscaLink::inquiry(const Packet & frame, Duration timeout)
{
  if (timeout.count() <= 0) timeout = timeout_;
  std::lock_guard<std::mutex> lock(mutex_);
  send(frame);
  // A bare command completion (y0 5z) has a one-byte payload; an inquiry reply
  // carries data, so insist on at least two bytes to skip stale completions.
  return await(visca::ReplyKind::Completion, timeout, 2);
}

std::vector<ViscaLink::Packet> ViscaLink::inquiries(const std::vector<Packet> & frames, Duration timeout)
{
  if (timeout.count() <= 0) timeout = timeout_;
  std::vector<Packet> out;
  out.reserve(frames.size());
  if (!pad_) {
    for (const auto & f : frames) {
      try {
        out.push_back(inquiry(f, timeout));
      } catch (const ViscaError &) {
        out.emplace_back();
      }
    }
    return out;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (desynced_) resync();
  Packet burst;
  for (const auto & f : frames) burst.insert(burst.end(), f.begin(), f.end());
  write(burst);
  pad();
  // Replies come back in the order asked. Each is a data-carrying completion
  // or an error; ACKs, bare completions and pad answers are skipped as usual.
  // The reply timeout, plus the time the questions and pads take on the wire
  // and a margin each -- not timeout x questions, which held viscaMutex_ (and
  // the state timer with it) for 13 s whenever a full poll went unanswered.
  const auto deadline = std::chrono::steady_clock::now() + timeout + Duration(60) * static_cast<int>(frames.size());
  while (out.size() < frames.size()) {
    Packet frame;
    try {
      frame = readFrame(deadline);
    } catch (const ViscaTimeout &) {
      desynced_ = true;
      throw;
    }
    const visca::Reply reply = visca::classify(frame);
    if (reply.kind == visca::ReplyKind::Error) {
      out.emplace_back();
    } else if (reply.kind == visca::ReplyKind::Completion && reply.payload.size() >= 2) {
      out.push_back(reply.payload);
    }
  }
  return out;
}

bool ViscaLink::probe(const std::string & port, int baud, uint8_t address, Duration timeout, int pad_replies)
{
  try {
    ViscaLink link(port, baud, address, timeout, pad_replies);
    for (int i = 0; i < 2; ++i) {
      const Packet payload = link.inquiry(visca::zoomPosInq(address));
      // A zoom reply is exactly 50 0p 0q 0r 0s: every data byte <= 0x0F. This
      // shape check is what MAVLink noise on a neighbouring port fails.
      if (payload.size() != 5 || payload[0] != 0x50) return false;
      for (size_t k = 1; k < 5; ++k) {
        if (payload[k] > 0x0F) return false;
      }
      const int pos = visca::parseNibbles(payload, 4);
      if (pos < 0 || pos > visca::ZOOM_DIGITAL_TELE_END) return false;
    }
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

std::pair<std::string, int> ViscaLink::autodetect(
  const std::vector<std::string> & ports, const std::vector<int> & bauds, uint8_t address,
  Duration timeout, int pad_replies)
{
  for (const auto & port : ports) {
    for (int baud : bauds) {
      if (probe(port, baud, address, timeout, pad_replies)) return {port, baud};
    }
  }
  return {"", 0};
}

std::vector<std::string> ViscaLink::defaultPortCandidates()
{
  std::vector<std::string> out;
  auto add = [&out](const std::string & p) {
    if (std::find(out.begin(), out.end(), p) == out.end()) out.push_back(p);
  };
  if (::access("/dev/fcb_visca", F_OK) == 0) add("/dev/fcb_visca");
  // The camera board's own ports next, VISCA interface first: found on the
  // first probe, and the autopilot's port is not sent VISCA bytes needlessly.
  for (auto & p : boardSerialPorts()) add(p);
  for (const char * pattern : {"/dev/ttyACM*", "/dev/ttyUSB*", "/dev/ttyTHS*"}) {
    for (auto & p : globPaths(pattern)) add(p);
  }
  return out;
}
}  // namespace fcb_camera
