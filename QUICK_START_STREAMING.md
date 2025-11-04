# 🚀 Quick Start: GStreamer Streaming (No ROS on Viewer!)

Stream your Raspberry Pi camera to any computer using GStreamer - **no ROS installation needed on the viewer!**

---

## 1️⃣ Rebuild Docker Image (One-time)

```bash
cd /home/aldajo92/ROS2_Docker_rpi_camera
docker build -t rpi_camera_ros2:latest .
```

---

## 2️⃣ Run Container on Raspberry Pi

```bash
docker run -it --privileged --network host rpi_camera_ros2:latest
```

---

## 3️⃣ Launch Camera with Streaming (Inside Container)

```bash
# Source all workspaces
sros2

# Launch with GStreamer streaming
ros2 launch waver_cv_bringup main_gscam_stream.launch.py
```

**The node will print a command like this:**
```
================================================================================
To view the stream on another computer, run:
================================================================================
gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink
================================================================================
```

---

## 4️⃣ Install GStreamer on Remote Computer (One-time)

**Ubuntu/Debian:**
```bash
sudo apt update
sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav
```

**macOS:**
```bash
brew install gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad gst-plugins-ugly gst-libav
```

**Windows:** Download from https://gstreamer.freedesktop.org/download/

---

## 5️⃣ View Stream on Remote Computer

Copy and paste the command from step 3! Example:

```bash
gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink
```

**That's it! 🎉 You should see your camera stream!**

---

## 📊 Launch Options

### Standard Resolution (640x480)
```bash
ros2 launch waver_cv_bringup main_gscam_stream.launch.py
```

### HD Resolution (1280x720)
```bash
ros2 launch waver_cv_bringup main_gscam_stream_hd.launch.py
```

### Custom Port
```bash
ros2 launch waver_cv_bringup main_gscam_stream.launch.py stream_port:=5555
```

### MJPEG Encoding (easier to decode)
```bash
ros2 launch waver_cv_bringup main_gscam_stream.launch.py stream_encoding:=mjpeg
```

---

## 🎨 Alternative Viewers (No Command Line!)

### VLC Media Player
1. Open VLC
2. Media → Open Network Stream
3. Enter: `udp://@:5000`
4. Click Play

### Python (No ROS!)
```python
import cv2

pipeline = "udpsrc port=5000 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! rtph264depay ! decodebin ! videoconvert ! appsink"
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

while True:
    ret, frame = cap.read()
    if ret:
        cv2.imshow('Camera', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

---

## 🔧 Troubleshooting

### Can't see stream?

1. **Check Raspberry Pi IP:**
   ```bash
   hostname -I
   ```

2. **Ping from remote computer:**
   ```bash
   ping <raspberry_pi_ip>
   ```

3. **Check firewall on Raspberry Pi:**
   ```bash
   sudo ufw allow 5000/udp
   ```

4. **Verify node is running:**
   ```bash
   ros2 node list | grep gstreamer
   ```

### Poor quality?

**Increase bitrate:**
```bash
ros2 launch waver_cv_bringup main_gscam_stream.launch.py stream_bitrate:=4000000
```

### Stuttering/freezing?

**Try MJPEG encoding:**
```bash
ros2 launch waver_cv_bringup main_gscam_stream.launch.py stream_encoding:=mjpeg
```

---

## 📚 Full Documentation

- **Complete Guide:** `GSTREAMER_STREAMING.md`
- **All Launch Options:** `ros2_ws/src/waver_cv_bringup/README.md`
- **Setup Guide:** `GSCAM2_SETUP.md`

---

## ✅ What You Get

- ✅ Stream from Raspberry Pi with ROS2
- ✅ View on any computer **without ROS**
- ✅ Low latency H.264 streaming
- ✅ Multiple encoding options (H.264, MJPEG, Raw)
- ✅ Works with VLC, Python OpenCV, FFmpeg, etc.
- ✅ Multiple viewers at the same time
- ✅ Standard protocols (UDP/RTP)

**Enjoy your ROS-free camera streaming! 🎥**

