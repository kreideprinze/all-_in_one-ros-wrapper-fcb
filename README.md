# All-in-One FCB ROS Wrapper

A ROS 2 driver for the **Sony FCB-EV9520L** zoom block camera. Plug the
camera in, launch one file, and you get:

- **Live video** as a compressed H.265 (or H.264) ROS topic: 1080p at 60 fps,
  small enough to record for hours.
- **Camera control as ROS services**: zoom, day/night (IR) mode, focus,
  exposure, white balance and more.
- **Camera status** (zoom level, IR mode, frame rate, ...) published twice a
  second.

It works with **either USB interface board** and figures out which one is
attached by itself. You don't need to configure anything for the board.

| Board | How it shows up | Notes |
|---|---|---|
| **Twiga USB3 NeoHD** | `USB3 NeoHD`, USB ID `04b4:00f9` | Works out of the box. |
| **Oppila LVDS-USB3** | `FX3 CAMERA`, USB ID `04b4:0040` | Needs its own 12 V supply; see [Oppila board setup](#oppila-board-setup). |

---

## Contents

1. [What you need](#1-what-you-need)
2. [Install](#2-install)
3. [Build](#3-build)
4. [Run it](#4-run-it)
5. [Zoom and day/night (feed) control](#5-zoom-and-daynight-feed-control)
6. [See the video](#6-see-the-video)
7. [Record and export to MP4](#7-record-and-export-to-mp4)
8. [Oppila board setup](#oppila-board-setup)
9. [Troubleshooting](#troubleshooting)
10. [Reference and credits](#reference-and-credits)

---

## 1. What you need

- **Camera:** a Sony FCB-EV9520L (or EV9500L) on a Twiga NeoHD or Oppila
  LVDS-USB3 board.
- **Computer:** Ubuntu 22.04 with **ROS 2 Humble**. Tested on an NVIDIA Jetson
  Orin (uses the hardware video encoder) and on an x86 laptop.
- **USB 3 port:** a blue one. At USB 2 speed the board cannot deliver 1080p
  video.

## 2. Install

Install the system packages once:

```bash
sudo apt update
sudo apt install ros-humble-camera-info-manager ros-humble-image-transport \
     ros-humble-rosbag2 ros-humble-rqt-image-view libavcodec-dev libavutil-dev libswscale-dev \
     v4l-utils ffmpeg python3-serial
# On a Jetson, for the hardware encoder:
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

Let your user open the camera's serial port, then **log out and back in**:

```bash
sudo usermod -aG dialout,video $USER
```

## 3. Build

This repository is a complete ROS 2 workspace. The three `ffmpeg_*` packages
it depends on are included as git submodules, so clone with
`--recurse-submodules`:

```bash
git clone --recurse-submodules https://github.com/kreideprinze/all-_in_one-ros-wrapper-fcb.git
cd all-_in_one-ros-wrapper-fcb
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

If you already cloned without `--recurse-submodules`, run
`git submodule update --init` first.

Building takes a few minutes the first time. In every new terminal, run
`source install/setup.bash` from this folder before using the driver.

**Optional:** give the camera stable device names (`/dev/fcb_video`,
`/dev/fcb_visca`). Plug the camera in first, then:

```bash
ros2 run fcb_camera fcb_udev_setup.sh
```

## 4. Run it

```bash
ros2 launch fcb_camera fcb_camera.launch.py
```

That's it: it finds the camera, its control port and the board type. A good
start looks like this (Oppila board shown):

```
VISCA camera on /dev/ttyACM2 @ 9600 baud (model 0x0711, rom 0x0103), padding replies out of the board
video autodetect: matched "FX3 CAMERA: FX3 CAMERA" (Oppila LVDS-USB3) -> /dev/video0
encoder gstreamer:nvv4l2h265enc ready (hevc;nv12;bgr8;bgr8), 8.0 Mbit/s cap, gop 30, quality 23
streaming 1920x1080 UYVY @ 60.00 fps from /dev/video0 ("FX3 CAMERA: FX3 CAMERA", Oppila LVDS-USB3)
```

Common options:

```bash
ros2 launch fcb_camera fcb_camera.launch.py codec:=h264              # H.264 instead of H.265
ros2 launch fcb_camera fcb_camera.launch.py bit_rate:=4000000        # 4 Mbit/s (smaller files)
ros2 launch fcb_camera fcb_camera.launch.py video_device:=/dev/video2 visca_port:=/dev/ttyACM0   # pick devices by hand
```

Check it is alive:

```bash
ros2 topic hz /fcb/image/ffmpeg           # should say ~60
ros2 topic echo --once /fcb/state         # zoom, IR mode, frame rate, ...
```

## 5. Zoom and day/night (feed) control

All control goes through ROS services under `/fcb`. Copy and paste:

**Zoom**

```bash
# Go to an exact zoom level (1x - 30x) and wait until the lens gets there
ros2 service call /fcb/set_zoom fcb_interfaces/srv/SetZoom "{mode: ratio, value: 10.0, wait: true}"

# Zoom in / out continuously (speed 0-7), then stop
ros2 service call /fcb/set_zoom fcb_interfaces/srv/SetZoom "{mode: tele, speed: 4}"
ros2 service call /fcb/set_zoom fcb_interfaces/srv/SetZoom "{mode: wide, speed: 4}"
ros2 service call /fcb/set_zoom fcb_interfaces/srv/SetZoom "{mode: stop}"

# Back to fully wide
ros2 service call /fcb/set_zoom fcb_interfaces/srv/SetZoom "{mode: ratio, value: 1.0}"
```

**Day / night (IR) feed**

| `mode` | What you get |
|---|---|
| `day` (also `rgb`, `color`) | Normal colour picture |
| `night` (also `ir`, `bw`) | Infrared-sensitive black and white, for low light |
| `night_color` | IR-sensitive but keeps colour |
| `auto` | Camera switches by itself depending on light |

```bash
ros2 service call /fcb/set_icr fcb_interfaces/srv/SetIcr "{mode: night}"
ros2 service call /fcb/set_icr fcb_interfaces/srv/SetIcr "{mode: day}"
ros2 service call /fcb/set_icr fcb_interfaces/srv/SetIcr "{mode: auto}"
```

**Other controls** (same pattern):

```bash
ros2 service call /fcb/set_focus fcb_interfaces/srv/SetFocus "{mode: one_push}"      # autofocus once
ros2 service call /fcb/set_exposure fcb_interfaces/srv/SetExposure "{mode: full_auto}"
ros2 service call /fcb/set_picture fcb_interfaces/srv/SetPicture "{stabilizer: 'on'}"
ros2 service call /fcb/preset fcb_interfaces/srv/Preset "{action: set, slot: 1}"     # save; later {action: recall, slot: 1}
```

Every service replies with `success` and a `message`. The zoom and IR
services also report the camera's new setting, read back from the camera.

From code, call the same services as you would any ROS service. The types
are in the `fcb_interfaces` package (`SetZoom`, `SetIcr`, ...).

## 6. See the video

The driver publishes compressed video. To view it on a screen:

```bash
ros2 launch fcb_camera view.launch.py                      # decodes to /fcb/image/decoded
ros2 run rqt_image_view rqt_image_view /fcb/image/decoded
```

## 7. Record and export to MP4

```bash
ros2 run fcb_camera fcb_record.sh my_flight                # Ctrl-C to stop; ~1 GB per 17 min
ros2 run fcb_camera fcb_bag_to_mp4.py my_flight -o my_flight.mp4
```

The export copies the video straight out of the bag without re-encoding,
so it is fast and loses no quality.

## Oppila board setup

The Oppila board works differently from the NeoHD board in a few ways. The
driver handles all of them automatically, but the board needs this setup:

1. **Power:** connect a **12 V / 2 A** supply to its 2-pin power connector
   (J9). The board also powers the camera from it. USB alone is not enough.
2. **Control switches:** set the **USB** camera-control switch **ON** and
   the **UART** switch **OFF**. Otherwise zoom and IR commands go to the
   board's wire header instead of over USB.
3. **Don't unplug USB while it is streaming.** If you do, the video will
   not come back until you **switch the 12 V supply off and on**. Rebooting
   the computer while the board stays powered counts as unplugging.

What the driver does for you on this board, so you know why the log looks
the way it does:

- **Control replies:** the board holds the camera's replies back until it
  has 32 bytes of them. The driver sends short harmless "filler" commands to
  push each reply out, hence "padding replies out of the board" in the log.
  Each reply takes about 0.2 s instead of a few milliseconds, which is fine
  for zoom and IR control.
- **Frame rate:** the board sends every 4th frame twice and reports the
  wrong frame rate. The driver drops the copies and records at the camera's
  real 60 fps, so videos play back at normal speed.
- **Video format:** the board sends UYVY instead of YUYV; the driver selects
  it automatically.

## Troubleshooting

| Problem | Fix |
|---|---|
| `camera not found` | Check `lsusb` lists `04b4:00f9` (NeoHD) or `04b4:0040` (Oppila). Use a blue USB 3 port and a USB 3 cable. |
| No video, or `no frames from /dev/videoN` | **Oppila:** switch its 12 V supply off and on (see above). **NeoHD:** unplug and replug it. |
| `no VISCA camera answered`: video works but zoom/IR services fail | Run `sudo usermod -aG dialout $USER` and log in again. **Oppila:** check the USB control switch is ON and UART is OFF. Make sure no other program has the camera open: `fuser /dev/ttyACM*`. |
| Kernel log (`sudo dmesg`) shows `error -71`, or the camera keeps disconnecting | That is a poor USB connection. Try another USB port or cable, preferably plugged directly into the computer rather than through a hub. |
| `Permission denied` on `/dev/video*` | `sudo usermod -aG video $USER`, then log in again. |
| Building fails on `ffmpeg_image_transport` | The submodules are missing: run `git submodule update --init`. |

## Reference and credits

- **Full reference:** every topic, service field, parameter and design note is
  in [docs/REFERENCE.md](docs/REFERENCE.md). Default settings are in
  [fcb_camera/config/fcb_camera.yaml](fcb_camera/config/fcb_camera.yaml).
- **Testing without a camera:** run `ros2 run fcb_camera fake_visca_camera.py`
  (add `--oppila` to imitate the Oppila board). See the reference for details.
- **Tests:** `colcon build` (without `-DBUILD_TESTING=OFF`), then
  `colcon test --packages-select fcb_camera && colcon test-result --verbose`.
- **Credits:** based on [AkshatKaushal25/fcb_ros](https://github.com/AkshatKaushal25/fcb_ros),
  extended with Oppila LVDS-USB3 board support. The video transport packages
  in `third_party/` are from
  [ros-misc-utilities](https://github.com/ros-misc-utilities).
- **License:** Apache-2.0, as declared by the packages.
