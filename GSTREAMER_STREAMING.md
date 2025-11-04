# GStreamer Streaming Without ROS

This guide shows how to stream camera images from your Raspberry Pi (running ROS2) to another computer using GStreamer, **without needing ROS installed on the remote computer**.

## Overview

The setup uses:
- **Raspberry Pi**: Runs ROS2 in Docker with gscam2 + custom GStreamer streaming node
- **Remote Computer**: Only needs GStreamer installed (no ROS required)
- **Network**: UDP/RTP streaming over local network

---

## Architecture

```
┌─────────────────────────────────────────┐
│  Raspberry Pi (Docker Container)       │
│                                         │
│  ┌──────────┐      ┌────────────────┐  │
│  │ gscam2   │─────>│ ROS2 /image_raw│  │
│  │          │      │     topic      │  │
│  └──────────┘      └────────┬───────┘  │
│                              │          │
│                              v          │
│                    ┌─────────────────┐  │
│                    │  gstreamer_     │  │
│                    │  streamer node  │  │
│                    └────────┬────────┘  │
│                              │          │
└──────────────────────────────┼──────────┘
                               │
                               │ UDP/RTP
                               │ Port 5000
                               │
                               v
                ┌──────────────────────────┐
                │  Remote Computer         │
                │                          │
                │  GStreamer receiver      │
                │  (no ROS needed!)        │
                └──────────────────────────┘
```

---

## Installation

### On Raspberry Pi (in Docker)

The necessary GStreamer packages are already included in the Dockerfile. When you rebuild, they will be installed automatically.

### On Remote Computer (Viewer)

Install GStreamer (no ROS required):

**Ubuntu/Debian:**
```bash
sudo apt update
sudo apt install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-x
```

**macOS (with Homebrew):**
```bash
brew install gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad gst-plugins-ugly gst-libav
```

**Windows:**
Download and install from: https://gstreamer.freedesktop.org/download/

---

## Usage

### Step 1: Rebuild Docker Image

Since we added GStreamer Python bindings and the new streaming node:

```bash
cd /home/aldajo92/ROS2_Docker_rpi_camera
docker build -t rpi_camera_ros2:latest .
```

### Step 2: Run Container on Raspberry Pi

Make sure to expose the streaming port (5000):

```bash
docker run -it \
  --privileged \
  --network host \
  rpi_camera_ros2:latest
```

Or if you prefer explicit port mapping:

```bash
docker run -it \
  --privileged \
  -p 5000:5000/udp \
  -p 8080:8080 \
  rpi_camera_ros2:latest
```

### Step 3: Launch Camera with Streaming (Inside Container)

**Option A: Standard Resolution (640x480) with H.264 streaming**
```bash
sros2  # Source all workspaces
ros2 launch waver_cv_bringup main_gscam_stream.launch.py
```

**Option B: HD Resolution (1280x720) with H.264 streaming**
```bash
sros2
ros2 launch waver_cv_bringup main_gscam_stream_hd.launch.py
```

**Option C: Custom settings**
```bash
sros2
ros2 launch waver_cv_bringup main_gscam_stream.launch.py \
  stream_port:=5000 \
  stream_encoding:=h264 \
  stream_bitrate:=2000000
```

When the node starts, it will print the **exact GStreamer command** to run on your remote computer!

### Step 4: View on Remote Computer

The node will print something like:

```
================================================================================
To view the stream on another computer, run:
================================================================================
gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink
================================================================================
```

**Copy and run that command on your remote computer!**

---

## Encoding Options

### H.264 (Default - Recommended)

**Pros:**
- Low bandwidth (~2 Mbps for 640x480)
- Good quality
- Low latency with proper settings

**Cons:**
- Requires CPU for encoding/decoding

**Launch with H.264:**
```bash
ros2 launch waver_cv_bringup main_gscam_stream.launch.py stream_encoding:=h264
```

**Receiver command (on remote computer):**
```bash
gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink
```

---

### MJPEG

**Pros:**
- Lower CPU usage than H.264
- Good compatibility
- Easy to decode

**Cons:**
- Higher bandwidth than H.264

**Launch with MJPEG:**
```bash
ros2 launch waver_cv_bringup main_gscam_stream.launch.py stream_encoding:=mjpeg
```

**Receiver command (on remote computer):**
```bash
gst-launch-1.0 udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! videoconvert ! autovideosink
```

---

### Raw (Uncompressed)

**Pros:**
- No encoding latency
- Maximum quality
- Lowest CPU usage

**Cons:**
- **Very high bandwidth** (~200+ Mbps for 640x480)
- Only suitable for local/high-speed networks

**Launch with Raw:**
```bash
ros2 launch waver_cv_bringup main_gscam_stream.launch.py stream_encoding:=raw
```

**Receiver command (on remote computer):**
```bash
gst-launch-1.0 udpsrc port=5000 ! application/x-rtp,media=video,encoding-name=RAW ! rtpvrawdepay ! videoconvert ! autovideosink
```

---

## Network Configuration

### Find Your Raspberry Pi IP Address

```bash
# On Raspberry Pi (outside container):
hostname -I

# Or inside container (if using --network host):
ip addr show
```

### Firewall Rules

Make sure UDP port 5000 is open on the Raspberry Pi:

```bash
# UFW (Ubuntu/Debian):
sudo ufw allow 5000/udp

# Firewalld (RHEL/CentOS):
sudo firewall-cmd --add-port=5000/udp --permanent
sudo firewall-cmd --reload
```

---

## Advanced Usage

### Change Streaming Port

```bash
ros2 launch waver_cv_bringup main_gscam_stream.launch.py stream_port:=5555
```

Then on remote computer:
```bash
gst-launch-1.0 udpsrc port=5555 ... (rest of command)
```

### Adjust Bitrate for HD Streaming

For better HD quality, increase bitrate:

```bash
ros2 launch waver_cv_bringup main_gscam_stream_hd.launch.py stream_bitrate:=8000000
```

### Save Stream to File (Remote Computer)

Instead of viewing, save to file:

**H.264 to MP4:**
```bash
gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! h264parse ! mp4mux ! filesink location=output.mp4
```

**MJPEG to AVI:**
```bash
gst-launch-1.0 udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! videoconvert ! avimux ! filesink location=output.avi
```

### Multiple Viewers

Multiple computers can receive the same stream simultaneously. Just run the receiver command on each computer.

---

## Troubleshooting

### No Stream Received

1. **Check network connectivity:**
   ```bash
   # On remote computer, ping Raspberry Pi:
   ping <raspberry_pi_ip>
   ```

2. **Verify port is open:**
   ```bash
   # On Raspberry Pi:
   sudo netstat -ulnp | grep 5000
   ```

3. **Check firewall:**
   - Make sure UDP port 5000 is open on both machines

4. **Verify node is running:**
   ```bash
   ros2 node list | grep gstreamer_streamer
   ```

### Poor Video Quality

1. **Increase bitrate:**
   ```bash
   ros2 launch waver_cv_bringup main_gscam_stream.launch.py stream_bitrate:=4000000
   ```

2. **Use MJPEG** for less compression:
   ```bash
   ros2 launch waver_cv_bringup main_gscam_stream.launch.py stream_encoding:=mjpeg
   ```

### High Latency

1. **Use H.264** (already optimized for low latency)
2. **Check network speed** - WiFi can add latency
3. **Use wired Ethernet** if possible

### Stream Freezes/Stutters

1. **Check CPU usage** on Raspberry Pi:
   ```bash
   top
   ```

2. **Lower resolution**:
   ```bash
   ros2 launch waver_cv_bringup main_gscam_stream.launch.py  # Use 640x480 instead of HD
   ```

3. **Check network bandwidth**:
   ```bash
   iperf3 -s  # On one machine
   iperf3 -c <ip_address>  # On the other
   ```

### GStreamer Errors on Remote Computer

1. **Install all plugins:**
   ```bash
   sudo apt install gstreamer1.0-plugins-*
   ```

2. **Check which plugins are available:**
   ```bash
   gst-inspect-1.0 | grep -i h264
   gst-inspect-1.0 | grep -i jpeg
   ```

---

## Comparison: Streaming Methods

| Method | Bandwidth | Latency | Quality | CPU Usage | ROS Required |
|--------|-----------|---------|---------|-----------|--------------|
| **GStreamer UDP (this guide)** | Low-Medium | Low | Good | Medium | ❌ Remote only |
| **web_video_server** | Medium | Medium | Good | Low | ❌ No (web browser) |
| **ROS image_transport** | Varies | Low | Excellent | Low | ✅ Both sides |

---

## Benefits of This Approach

✅ **No ROS needed on viewer computer** - just GStreamer  
✅ **Low latency** - UDP streaming with H.264  
✅ **Flexible encoding** - H.264, MJPEG, or Raw  
✅ **Multiple viewers** - broadcast to many computers  
✅ **Standard protocol** - works with any GStreamer receiver  
✅ **Easy to integrate** - can pipe into other applications  

---

## Integration Examples

### View in Python (Remote Computer - No ROS)

```python
import cv2
import subprocess

# Start GStreamer pipeline
pipeline = "udpsrc port=5000 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! rtph264depay ! decodebin ! videoconvert ! appsink"

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

while True:
    ret, frame = cap.read()
    if ret:
        cv2.imshow('RPI Camera Stream', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
```

### View in VLC Media Player

1. Open VLC
2. Media → Open Network Stream
3. Enter: `udp://@:5000`
4. Click Play

### View in FFmpeg/FFplay

```bash
ffplay -fflags nobuffer -flags low_delay -framedrop -strict experimental "udp://0.0.0.0:5000"
```

---

## Summary

This setup allows you to:
- Run ROS2 and gscam2 on Raspberry Pi
- Stream camera via GStreamer UDP/RTP
- View on **any computer** without installing ROS
- Choose encoding (H.264, MJPEG, or Raw)
- Integrate with standard video tools (VLC, FFmpeg, Python OpenCV, etc.)

**Next steps:**
1. Rebuild Docker image with new GStreamer support
2. Launch with `main_gscam_stream.launch.py`
3. Copy the receiver command and run on remote computer
4. Enjoy low-latency camera streaming! 🎥

