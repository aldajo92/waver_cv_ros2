# Waver CV Bringup

Launch files for bringing up the camera system with computer vision processing and web video streaming.

## Available Launch Files

### 1. `main.launch.py` (Original - camera_ros)
Uses the `camera_ros` package with libcamera backend.

**Usage:**
```bash
ros2 launch waver_cv_bringup main.launch.py
```

**Features:**
- Camera resolution: 640x480
- Format: BGR888
- Includes simple_sub_pub node for OpenCV processing
- Includes web_video_server for remote viewing

---

### 2. `main_gscam.launch.py` (New - gscam2)
Uses `gscam2` with GStreamer pipeline for Raspberry Pi camera via libcamera.

**Usage:**
```bash
ros2 launch waver_cv_bringup main_gscam.launch.py
```

**Features:**
- Camera resolution: 640x480 @ 30fps
- Uses libcamerasrc GStreamer element
- Publishes to `/image_raw` and `/camera_info` topics
- Includes simple_sub_pub node for OpenCV processing
- Includes web_video_server for remote viewing

**GStreamer Pipeline:**
```
libcamerasrc ! video/x-raw,width=640,height=480,format=BGR ! videoconvert
```

---

### 3. `main_gscam_hd.launch.py` (HD Resolution - gscam2)
Uses `gscam2` with HD resolution (1280x720).

**Usage:**
```bash
ros2 launch waver_cv_bringup main_gscam_hd.launch.py
```

**Optional Arguments:**
```bash
ros2 launch waver_cv_bringup main_gscam_hd.launch.py width:=1920 height:=1080 framerate:=30
```
*(Note: The GStreamer pipeline currently uses fixed HD resolution, but arguments are prepared for future dynamic configuration)*

**Features:**
- Camera resolution: 1280x720 @ 30fps (HD)
- Uses libcamerasrc GStreamer element
- Higher resolution for better image quality
- Includes simple_sub_pub node for OpenCV processing
- Includes web_video_server for remote viewing

---

### 4. `main_gscam_stream.launch.py` (GStreamer Streaming - No ROS needed on viewer!)
Uses `gscam2` + custom GStreamer streaming node to broadcast video over UDP.

**Usage:**
```bash
ros2 launch waver_cv_bringup main_gscam_stream.launch.py
```

**Features:**
- Camera resolution: 640x480 @ 30fps
- **GStreamer UDP/RTP streaming** - view on any computer without ROS!
- Default port: 5000 (UDP)
- Default encoding: H.264 (low latency, good quality)
- Includes simple_sub_pub node for OpenCV processing
- Includes web_video_server for remote viewing

**Optional Arguments:**
```bash
ros2 launch waver_cv_bringup main_gscam_stream.launch.py \
  stream_port:=5000 \
  stream_encoding:=h264 \
  stream_bitrate:=2000000
```

**Encoding Options:**
- `h264` - Low bandwidth, good quality (default)
- `mjpeg` - Medium bandwidth, easy to decode
- `raw` - High bandwidth, no latency

**View on Remote Computer (No ROS required!):**

When you launch, the node prints the exact command. Example:
```bash
# On remote computer with GStreamer installed:
gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink
```

📖 **See `GSTREAMER_STREAMING.md` in project root for complete guide!**

---

### 5. `main_gscam_stream_hd.launch.py` (HD GStreamer Streaming)
Same as above but with HD resolution (1280x720).

**Usage:**
```bash
ros2 launch waver_cv_bringup main_gscam_stream_hd.launch.py
```

**Features:**
- Camera resolution: 1280x720 @ 30fps (HD)
- GStreamer UDP/RTP streaming
- Higher bitrate for HD quality (4 Mbps default)
- View on any computer without ROS!

---

## Configuration Files

### `config/rpi_camera_params.yaml`
Parameter file for gscam2 with Raspberry Pi camera configuration.

You can modify this file to change camera settings like:
- Resolution (width/height)
- Framerate
- GStreamer pipeline
- Camera calibration file path

**Example of custom launch with config file:**
```python
from ament_index_python.packages import get_package_share_directory
import os

config_file = os.path.join(
    get_package_share_directory('waver_cv_bringup'),
    'config',
    'rpi_camera_params.yaml'
)

Node(
    package='gscam2',
    executable='gscam_main',
    parameters=[config_file]
)
```

---

## Nodes Launched

All launch files start these nodes:

1. **Camera Node**: Either `camera_ros` or `gscam2` (depending on launch file)
   - Publishes: `/image_raw`, `/camera_info`

2. **simple_sub_pub**: From `waver_cv` package
   - Subscribes to camera images
   - Performs OpenCV processing
   - Publishes processed images

3. **web_video_server**: Video streaming server
   - Access via web browser at `http://<robot_ip>:8080`
   - Provides MJPEG, VP8, and other streaming formats

---

## Testing GStreamer Pipeline

You can test the GStreamer pipeline directly before launching:

```bash
# Test if libcamerasrc works
gst-launch-1.0 libcamerasrc ! video/x-raw,width=640,height=480 ! videoconvert ! autovideosink

# Test with specific format (what gscam2 will use)
gst-launch-1.0 libcamerasrc ! video/x-raw,width=640,height=480,format=BGR ! videoconvert ! autovideosink
```

---

## Viewing the Camera Stream

### Via ROS2 Tools:
```bash
# List topics
ros2 topic list

# Echo camera info
ros2 topic echo /camera_info

# View image with rqt
rqt_image_view
```

### Via Web Browser:
Navigate to: `http://<robot_ip>:8080`

---

## Troubleshooting

### Camera not detected:
- Make sure libcamera is properly installed
- Test with: `libcamera-hello`
- Check camera permissions

### GStreamer errors:
- Verify GStreamer plugins are installed: `gst-inspect-1.0 libcamerasrc`
- Check GStreamer pipeline manually (see Testing section above)

### No image on topic:
- Check if the camera node is running: `ros2 node list`
- Check if topics are being published: `ros2 topic hz /image_raw`
- Check node logs: `ros2 node info /rpi_camera/gscam_publisher`

