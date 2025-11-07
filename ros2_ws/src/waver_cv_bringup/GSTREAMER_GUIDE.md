# GStreamer Processing Guide

You want: **Camera → GStreamer (process) → ROS (publish already processed)**

## Quick Start - Use Built-in Filters

Edit `launch/main_gscam.launch.py` line 11:

```python
# Original (no processing)
gscam_config = 'v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480 ! videoconvert ! video/x-raw,format=RGB'

# Edge detection
gscam_config = 'v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480 ! videoconvert ! edgetv ! videoconvert ! video/x-raw,format=RGB'

# Grayscale
gscam_config = 'v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480 ! videoconvert ! video/x-raw,format=GRAY8 ! videoconvert ! video/x-raw,format=RGB'

# Brightness/Contrast
gscam_config = 'v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480 ! videoconvert ! videobalance brightness=0.3 contrast=1.5 ! videoconvert ! video/x-raw,format=RGB'
```

Then rebuild and run:
```bash
./scripts/build.sh
./scripts/run.sh
```

## Available GStreamer Filters

Check what's installed in your container:
```bash
./scripts/shell.sh

# Inside container:
gst-inspect-1.0 | grep -i filter
gst-inspect-1.0 videobalance  # See parameters
gst-inspect-1.0 edgetv
```

Common filters:
- `edgetv` - Edge detection
- `videobalance` - Brightness, contrast, saturation
- `gamma` - Gamma correction
- `videodetect` - Motion detection
- `facedetect` - Face detection (if opencv plugin installed)
- `videoflip` - Flip/rotate
- `videoscale` - Resize

## More Examples

See `launch/gscam_examples.py` for 10+ ready-to-use examples.

## Advanced: Python OpenCV in Pipeline

See `scripts/gst_python_processor.py` for standalone example.

This uses `appsink` → Python OpenCV → `appsrc` to process frames
in the pipeline without ROS topics.

