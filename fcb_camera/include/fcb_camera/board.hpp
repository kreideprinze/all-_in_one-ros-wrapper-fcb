// Which USB interface board the camera is on, and how that board behaves.
//
// The FCB-EV9520L reaches the host through one of two boards, told apart by
// USB ID because the V4L2 card name of the second is just "FX3 CAMERA":
//
//   Twiga "USB3 NeoHD"   04b4:00f9 (04b4:00f8 when the link falls back to
//                        USB 2). YUYV. One CDC-ACM port, which is VISCA.
//
//   Oppila LVDS-USB3     04b4:0040, card "FX3 CAMERA". UYVY only. Two CDC-ACM
//                        ports: interface 0 is VISCA at 9600 baud, the other
//                        is debug output from the board's USB controller.
//                        Measured on the drone, it also
//                          - passes VISCA replies to the host only in full
//                            32-byte USB packets, so a lone 3-7 byte reply
//                            stays in the board (ViscaLink pads to push it out);
//                          - delivers 78.7 frames/s for the camera's 60, every
//                            fourth a byte-for-byte repeat of the third, and
//                            reports 30 fps whatever is requested;
//                          - does not recover video from a USB reset or replug
//                            until its 12 V supply is cycled (Oppila's docs).
#pragma once

#include <string>
#include <vector>

namespace fcb_camera
{
enum class Board { Unknown, NeoHD, Oppila };

struct BoardProfile
{
  Board board{Board::Unknown};
  std::string name{"unknown board"};
  // Native packed 4:2:2 format; "" when the board is unknown.
  std::string fourcc;
  bool pads_visca_replies{false};
  bool repeats_frames{false};
  bool usb_reset_recovers{true};
};

// "vvvv:pppp" of the USB device behind a /dev node (symlinks resolved), or "".
std::string usbIdOf(const std::string & dev_path);
// bInterfaceNumber of the USB interface behind a /dev node, or -1.
int usbInterfaceOf(const std::string & dev_path);

BoardProfile boardForUsbId(const std::string & usb_id);
BoardProfile boardFor(const std::string & dev_path);

// /dev/ttyACM* nodes on a known camera board, the VISCA interface first. The
// Oppila board's debug port sorts after its VISCA port, so a sweep finds the
// camera on the first probe.
std::vector<std::string> boardSerialPorts();
}  // namespace fcb_camera
