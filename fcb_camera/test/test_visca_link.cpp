// ViscaLink against a simulated camera on a pseudo-terminal, including one that
// behaves like the Oppila LVDS-USB3 board: replies leave it only in full 32-byte
// packets, and broadcast Address Set (88 30 01 FF) is answered with 88 30 02 FF.
#include <gtest/gtest.h>

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "fcb_camera/board.hpp"
#include "fcb_camera/visca_link.hpp"

using namespace fcb_camera;
using Ms = std::chrono::milliseconds;

namespace
{
class FakeCamera
{
public:
  explicit FakeCamera(bool chunked) : chunked_(chunked)
  {
    int slave = -1;
    char name[256];
    if (::openpty(&master_, &slave, name, nullptr, nullptr) != 0) throw std::runtime_error("openpty");
    path_ = name;
    ::close(slave);
    thread_ = std::thread([this]() { run(); });
  }
  ~FakeCamera()
  {
    running_ = false;
    thread_.join();
    ::close(master_);
  }
  const std::string & path() const { return path_; }
  std::atomic<int> zoom{0x2063};
  std::atomic<bool> silent{false};

private:
  void send(const visca::Packet & p)
  {
    if (!chunked_) {
      (void)!::write(master_, p.data(), p.size());
      return;
    }
    held_.insert(held_.end(), p.begin(), p.end());
    while (held_.size() >= 32) {
      (void)!::write(master_, held_.data(), 32);
      held_.erase(held_.begin(), held_.begin() + 32);
    }
  }
  void respond(const visca::Packet & f)
  {
    if (silent) return;
    if (f == visca::Packet{0x88, 0x30, 0x01, 0xFF}) {
      if (chunked_) send({0x88, 0x30, 0x02, 0xFF});
      return;
    }
    if (f == visca::zoomPosInq(1)) {
      const int z = zoom;
      send({0x90, 0x50, uint8_t((z >> 12) & 15), uint8_t((z >> 8) & 15), uint8_t((z >> 4) & 15),
            uint8_t(z & 15), 0xFF});
    } else if (f.size() >= 4 && f[1] == 0x09) {
      send({0x90, 0x50, 0x03, 0xFF});  // any other inquiry: a one-byte answer
    } else if (f == visca::ifClear(1)) {
      send({0x90, 0x50, 0xFF});
    } else {
      send({0x90, 0x41, 0xFF});
      send({0x90, 0x51, 0xFF});
    }
  }
  void run()
  {
    visca::Packet frame;
    while (running_) {
      pollfd p{master_, POLLIN, 0};
      if (::poll(&p, 1, 20) <= 0) continue;
      uint8_t b;
      if (::read(master_, &b, 1) != 1) continue;
      frame.push_back(b);
      if (b != 0xFF) continue;
      respond(frame);
      frame.clear();
    }
  }
  bool chunked_;
  int master_{-1};
  std::string path_;
  visca::Packet held_;
  std::atomic<bool> running_{true};
  std::thread thread_;
};
}  // namespace

TEST(Board, KnownBoardsByUsbId)
{
  EXPECT_EQ(boardForUsbId("04b4:00f9").board, Board::NeoHD);
  EXPECT_EQ(boardForUsbId("04b4:00f8").board, Board::NeoHD);
  EXPECT_EQ(boardForUsbId("04b4:00f9").fourcc, "YUYV");
  EXPECT_FALSE(boardForUsbId("04b4:00f9").pads_visca_replies);
  EXPECT_FALSE(boardForUsbId("04b4:00f9").repeats_frames);

  const auto oppila = boardForUsbId("04b4:0040");
  EXPECT_EQ(oppila.board, Board::Oppila);
  EXPECT_EQ(oppila.fourcc, "UYVY");
  EXPECT_TRUE(oppila.pads_visca_replies);
  EXPECT_TRUE(oppila.repeats_frames);
  EXPECT_FALSE(oppila.usb_reset_recovers);

  EXPECT_EQ(boardForUsbId("04b4:1234").board, Board::Unknown);  // some other FX3 device
  EXPECT_EQ(boardForUsbId("").board, Board::Unknown);
  EXPECT_EQ(boardFor("/dev/pts/0").board, Board::Unknown);       // not USB at all
}

TEST(ViscaLink, PlainCameraUnchanged)
{
  FakeCamera cam(false);
  ViscaLink link(cam.path(), 9600, 1, Ms(500));
  EXPECT_FALSE(link.padsReplies());
  EXPECT_EQ(visca::parseNibbles(link.inquiry(visca::zoomPosInq(1)), 4), 0x2063);
  link.command(visca::zoomStop(1));
}

TEST(ViscaLink, HeldBackRepliesArePaddedOut)
{
  FakeCamera cam(true);
  ViscaLink link(cam.path(), 9600, 1, Ms(500), 1);
  EXPECT_TRUE(link.padsReplies());
  // Repeated, so the replies land at different offsets in the board's packets.
  for (int z : {0x2063, 0x0000, 0x4000, 0x16A1, 0x3486}) {
    cam.zoom = z;
    EXPECT_EQ(visca::parseNibbles(link.inquiry(visca::zoomPosInq(1)), 4), z);
  }
  link.command(visca::zoomStop(1));  // its ACK is held back too
  link.commandNoAck(visca::ifClear(1));
}

TEST(ViscaLink, WithoutPaddingTheReplyNeverArrives)
{
  FakeCamera cam(true);
  ViscaLink link(cam.path(), 9600, 1, Ms(300), 0);
  EXPECT_THROW(link.inquiry(visca::zoomPosInq(1)), ViscaTimeout);
}

TEST(ViscaLink, BatchedInquiriesComeBackInOrder)
{
  for (bool chunked : {false, true}) {
    FakeCamera cam(chunked);
    ViscaLink link(cam.path(), 9600, 1, Ms(500), chunked ? 1 : 0);
    cam.zoom = 0x16A1;
    const auto r = link.inquiries({visca::zoomPosInq(1), visca::icrInq(1), visca::zoomPosInq(1)});
    ASSERT_EQ(r.size(), 3u);
    EXPECT_EQ(visca::parseNibbles(r[0], 4), 0x16A1);
    EXPECT_EQ(visca::parseByte(r[1]), 0x03);
    EXPECT_EQ(visca::parseNibbles(r[2], 4), 0x16A1);
    // and the link is still in step afterwards
    EXPECT_EQ(visca::parseNibbles(link.inquiry(visca::zoomPosInq(1)), 4), 0x16A1);
  }
}

TEST(ViscaLink, ALateReplyIsNotTakenForTheNextAnswer)
{
  FakeCamera cam(true);
  ViscaLink link(cam.path(), 9600, 1, Ms(500), 1);
  cam.zoom = 0x1111;
  link.inquiry(visca::zoomPosInq(1));
  cam.silent = true;
  EXPECT_THROW(link.inquiry(visca::zoomPosInq(1)), ViscaTimeout);
  cam.silent = false;
  cam.zoom = 0x2222;
  EXPECT_EQ(visca::parseNibbles(link.inquiry(visca::zoomPosInq(1)), 4), 0x2222);
}
