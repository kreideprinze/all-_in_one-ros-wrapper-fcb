// Serial transport for VISCA: exclusive, framed, and safe to share between threads.
//
// Every VISCA packet ends in 0xFF. A command is answered with an ACK (y0 4z FF)
// once accepted and a separate completion (y0 5z FF) once the action finished,
// which for a zoom move can be seconds later. `command()` therefore waits only
// for the ACK by default; late completions are skipped by the next exchange.
// Inquiries are answered with a single data-carrying completion (y0 50 .. FF).
//
// Some boards hold replies back: the Oppila LVDS-USB3 board passes camera bytes
// to the host only in full 32-byte USB packets, so a lone reply sits in the
// board until later ones fill the packet. On such a link every packet is
// followed by broadcast Address Set "pads" (88 30 01 FF), whose 4-byte answers
// (88 30 02 FF) fill the packet and push the real reply out. Address Set only
// renumbers cameras, and the one camera on the bus is already address 1; the
// answers classify as Unknown and are skipped like any stale frame.
#pragma once

#include <chrono>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "fcb_camera/visca.hpp"

namespace fcb_camera
{
class ViscaError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};
class ViscaTimeout : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class ViscaLink
{
public:
  using Packet = visca::Packet;
  using Duration = std::chrono::milliseconds;

  // Opens the port at `baud`, takes an exclusive flock. Throws std::runtime_error.
  // `pad_replies`: 1 on, 0 off, -1 decide from the port's board (see board.hpp).
  ViscaLink(const std::string & port, int baud, uint8_t address, Duration timeout, int pad_replies = -1);
  ~ViscaLink();
  ViscaLink(const ViscaLink &) = delete;
  ViscaLink & operator=(const ViscaLink &) = delete;

  const std::string & port() const { return port_; }
  int baud() const { return baud_; }
  uint8_t address() const { return address_; }
  bool padsReplies() const { return pad_; }

  // Send a command frame; wait for ACK, and for the completion too if requested.
  void command(const Packet & frame, bool wait_completion = false, Duration timeout = Duration(0));
  // Send a command that is answered with a bare completion only (IF_Clear).
  void commandNoAck(const Packet & frame, Duration timeout = Duration(0));
  // Send an inquiry and return the completion payload (0x50 <data...>).
  Packet inquiry(const Packet & frame, Duration timeout = Duration(0));
  // Several inquiries; replies in the same order, an empty Packet where the
  // camera answered with an error. On a padding link they go out together
  // behind one round of pads -- one round trip for the lot instead of one
  // each, which on the Oppila board is ~0.2 s against ~0.2 s per question.
  // Elsewhere they are asked one at a time, exactly as inquiry() would.
  std::vector<Packet> inquiries(const std::vector<Packet> & frames, Duration timeout = Duration(0));

  // Convenience: true when the port answers a zoom inquiry twice with a well-formed reply.
  static bool probe(const std::string & port, int baud, uint8_t address, Duration timeout,
                    int pad_replies = -1);
  // Sweep candidate ports and bauds. Returns {port, baud} or {"", 0}.
  // A padded port gets at least 1 s per reply whatever `timeout` says.
  static std::pair<std::string, int> autodetect(
    const std::vector<std::string> & ports, const std::vector<int> & bauds, uint8_t address,
    Duration timeout, int pad_replies = -1);
  static std::vector<std::string> defaultPortCandidates();

private:
  Packet readFrame(std::chrono::steady_clock::time_point deadline);
  Packet await(visca::ReplyKind want, Duration timeout, size_t min_payload);
  // On a padding link, paced one packet at a time to the board's serial rate.
  void write(const Packet & frame);
  void writeRaw(const Packet & frame);
  // write() plus, on a padding link, the pads that push its reply out.
  void send(const Packet & frame);
  void pad();
  // Drop whatever the board still holds from before (an earlier owner of the
  // port, or a reply that came too late): pads out, read until their answers.
  void resync();
  void flushInput();

  std::string port_;
  int baud_;
  uint8_t address_;
  Duration timeout_;
  int fd_{-1};
  bool pad_{false};
  bool desynced_{false};
  std::chrono::steady_clock::time_point paddedAt_{};
  std::mutex mutex_;
};
}  // namespace fcb_camera
