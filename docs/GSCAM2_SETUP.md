# GScam2 Setup Guide

## What Was Created

### New Launch Files
1. **`main_gscam.launch.py`** - Standard resolution (640x480) with gscam2
2. **`main_gscam_hd.launch.py`** - HD resolution (1280x720) with gscam2

### Configuration
- **`config/rpi_camera_params.yaml`** - Parameter file for easy camera configuration

### Documentation
- **`ros2_ws/src/waver_cv_bringup/README.md`** - Complete usage guide

### Files Modified
- **`setup.py`** - Updated to include config directory
- **`Dockerfile`** - Fixed dependency issues:
  - Added ros2_shared workspace
  - Fixed ENV format warnings
  - Removed invalid opencv-python dependency
  - Added ros2_shared to skip-keys

---

## Rebuild Docker Image

After these changes, you need to rebuild the Docker image:

```bash
# Navigate to project directory
cd /home/aldajo92/ROS2_Docker_rpi_camera

# Rebuild Docker image
docker build -t rpi_camera_ros2:latest .

# Or with a specific name
docker build -t waver_rpi_camera:humble .
```

---

## Running the Container

```bash
# Run with camera access and network
docker run -it \
  --device /dev/video0 \
  --privileged \
  -p 8080:8080 \
  -v /run/udev:/run/udev:ro \
  rpi_camera_ros2:latest
```

---

## Inside the Container

### Source the workspaces:
```bash
# Use the convenient alias
sros2

# Or manually:
source /opt/ros/humble/setup.bash
source /ros2_shared_ws/install/setup.bash
source /camera_ws/install/setup.bash
source /ros2_ws/install/setup.bash
```

### Launch with original camera_ros:
```bash
ros2 launch waver_cv_bringup main.launch.py
```

### Launch with gscam2 (standard resolution):
```bash
ros2 launch waver_cv_bringup main_gscam.launch.py
```

### Launch with gscam2 (HD resolution):
```bash
ros2 launch waver_cv_bringup main_gscam_hd.launch.py
```

---

## Key Differences: camera_ros vs gscam2

| Feature | camera_ros | gscam2 |
|---------|-----------|--------|
| **Backend** | libcamera API directly | GStreamer with libcamerasrc |
| **Flexibility** | Limited to libcamera features | Full GStreamer pipeline power |
| **Configuration** | Python launch parameters | GStreamer config string |
| **Performance** | Direct API access | Extra GStreamer layer |
| **Use Case** | Simple camera access | Complex pipelines, effects, encoding |

---

## GStreamer Pipeline Examples

### Standard (what main_gscam.launch.py uses):
```
libcamerasrc ! video/x-raw,width=640,height=480,format=BGR ! videoconvert
```

### HD Resolution:
```
libcamerasrc ! video/x-raw,width=1280,height=720,framerate=30/1,format=BGR ! videoconvert
```

### With Image Processing:
```
libcamerasrc ! video/x-raw,width=640,height=480 ! videoconvert ! videoflip method=horizontal-flip ! video/x-raw,format=BGR ! videoconvert
```

### With H264 Encoding:
```
libcamerasrc ! video/x-raw,width=1920,height=1080 ! videoconvert ! x264enc tune=zerolatency ! rtph264pay
```

---

## Viewing the Stream

### Web Browser:
Navigate to: `http://<raspberry_pi_ip>:8080`

### ROS2 Tools:
```bash
# In another terminal (inside or outside container)
ros2 topic list
ros2 topic hz /image_raw
rqt_image_view
```

---

## Troubleshooting

### Issue: "Cannot locate rosdep definition for [ros2_shared]"
**Solution:** This is fixed! The ros2_shared package is now built in a separate workspace and added to skip-keys.

### Issue: "Cannot locate rosdep definition for [opencv-python]"
**Solution:** This is fixed! Removed from package.xml as OpenCV is installed via system packages.

### Issue: Camera not accessible in container
**Solution:** Run container with `--privileged` flag and mount camera devices.

### Issue: GStreamer plugin not found
**Solution:** Ensure GStreamer plugins are installed in Dockerfile (they are in the current setup).

---

## Next Steps

1. ✅ Rebuild Docker image
2. ✅ Test with standard resolution: `ros2 launch waver_cv_bringup main_gscam.launch.py`
3. ✅ Test with HD resolution: `ros2 launch waver_cv_bringup main_gscam_hd.launch.py`
4. 📝 Customize GStreamer pipeline in `config/rpi_camera_params.yaml` if needed
5. 📝 Add camera calibration file if you have one
6. 📝 Adjust resolution/framerate based on your Raspberry Pi camera model

---

## Camera Support

This setup should work with:
- Raspberry Pi Camera Module v1
- Raspberry Pi Camera Module v2
- Raspberry Pi Camera Module v3
- Raspberry Pi HQ Camera
- Compatible third-party cameras

Make sure your camera is properly connected and detected:
```bash
# On Raspberry Pi (outside container):
libcamera-hello --list-cameras
```

