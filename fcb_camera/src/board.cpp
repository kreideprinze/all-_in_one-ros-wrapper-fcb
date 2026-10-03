#include "fcb_camera/board.hpp"

#include <glob.h>
#include <limits.h>
#include <stdlib.h>

#include <algorithm>
#include <fstream>
#include <tuple>

namespace fcb_camera
{
namespace
{
std::string readTrimmed(const std::string & path)
{
  std::ifstream in(path);
  std::string s;
  if (!(in >> s)) return "";
  return s;
}

std::string realPath(const std::string & path)
{
  char buf[PATH_MAX];
  return ::realpath(path.c_str(), buf) ? std::string(buf) : std::string();
}

// sysfs directory of the USB interface behind a /dev node, or "".
std::string usbInterfaceDir(const std::string & dev_path)
{
  const std::string resolved = realPath(dev_path);
  if (resolved.empty()) return "";
  const std::string name = resolved.substr(resolved.find_last_of('/') + 1);
  for (const char * subsystem : {"video4linux", "tty"}) {
    const std::string dir = realPath(std::string("/sys/class/") + subsystem + "/" + name + "/device");
    if (!dir.empty()) return dir;
  }
  return "";
}
}  // namespace

std::string usbIdOf(const std::string & dev_path)
{
  const std::string iface = usbInterfaceDir(dev_path);
  if (iface.empty()) return "";
  const std::string device = iface.substr(0, iface.find_last_of('/'));
  const std::string vendor = readTrimmed(device + "/idVendor");
  const std::string product = readTrimmed(device + "/idProduct");
  if (vendor.empty() || product.empty()) return "";
  return vendor + ":" + product;
}

int usbInterfaceOf(const std::string & dev_path)
{
  const std::string iface = usbInterfaceDir(dev_path);
  if (iface.empty()) return -1;
  const std::string n = readTrimmed(iface + "/bInterfaceNumber");
  if (n.empty()) return -1;
  try {
    return std::stoi(n, nullptr, 16);
  } catch (...) {
    return -1;
  }
}

BoardProfile boardForUsbId(const std::string & usb_id)
{
  BoardProfile p;
  if (usb_id == "04b4:00f9" || usb_id == "04b4:00f8") {
    p.board = Board::NeoHD;
    p.name = usb_id == "04b4:00f8" ? "Twiga USB3 NeoHD (on a USB 2 link)" : "Twiga USB3 NeoHD";
    p.fourcc = "YUYV";
  } else if (usb_id == "04b4:0040") {
    p.board = Board::Oppila;
    p.name = "Oppila LVDS-USB3";
    p.fourcc = "UYVY";
    p.pads_visca_replies = true;
    p.repeats_frames = true;
    p.usb_reset_recovers = false;
  }
  return p;
}

BoardProfile boardFor(const std::string & dev_path) { return boardForUsbId(usbIdOf(dev_path)); }

std::vector<std::string> boardSerialPorts()
{
  std::vector<std::tuple<int, std::string>> found;
  glob_t g{};
  if (::glob("/dev/ttyACM*", 0, nullptr, &g) == 0) {
    for (size_t i = 0; i < g.gl_pathc; ++i) {
      const std::string port = g.gl_pathv[i];
      if (boardFor(port).board == Board::Unknown) continue;
      const int iface = usbInterfaceOf(port);
      found.emplace_back(iface < 0 ? 0 : iface, port);
    }
  }
  globfree(&g);
  std::sort(found.begin(), found.end());
  std::vector<std::string> out;
  for (auto & [iface, port] : found) out.push_back(port);
  return out;
}
}  // namespace fcb_camera
