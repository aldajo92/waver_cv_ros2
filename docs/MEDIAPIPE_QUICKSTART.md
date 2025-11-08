# MediaPipe Face Detection - Quick Start

## Step 1: Install MediaPipe in Docker

```bash
# Enter Docker container
./scripts/shell.sh

# Inside container, run:
/ros2_ws/src/waver_cv_bringup/scripts/install_mediapipe.sh
```

Or manually:
```bash
pip3 install mediapipe opencv-python
```

## Step 2: Test Face Detection (Standalone)

```bash
# Inside Docker container:
cd /ros2_ws/src/waver_cv_bringup/scripts
python3 gst_mediapipe_face.py
```

This opens a window showing face detection. Press Ctrl+C to stop.

## Step 3: Integrate with ROS (Advanced)

The script processes video in GStreamer BEFORE it goes to ROS.

### Architecture:
```
Camera → GStreamer → appsink → MediaPipe (Python) → appsrc → gscam2 → ROS
```

### For production, you can:

1. **Option A**: Run as separate process and use GStreamer inter-pipes
2. **Option B**: Embed in a ROS node that uses appsink/appsrc
3. **Option C**: Create custom GStreamer element (most efficient)

## How It Works

1. **Camera capture**: v4l2src grabs frames from /dev/video0
2. **appsink**: Extracts frames to Python
3. **MediaPipe**: Detects faces and draws bounding boxes
4. **appsrc**: Sends processed frames back to pipeline
5. **ROS**: gscam2 publishes already-processed frames

## Customization

Edit `gst_mediapipe_face.py`:

```python
# Change detection model
model_selection=0  # 0 = short-range (< 2m), 1 = full-range

# Change confidence threshold  
min_detection_confidence=0.5  # Lower = more detections, higher = fewer false positives

# Add more processing
if results.detections:
    for detection in results.detections:
        # Your custom code here
        print(f"Face detected with confidence: {detection.score[0]}")
```

## Other MediaPipe Solutions Available

- `gst_mediapipe_hands.py` - Hand tracking
- Face Mesh - 468 facial landmarks
- Pose Detection - Body pose estimation  
- Holistic - Face + hands + pose together
- Object Detection
- Segmentation

## Performance Tips

- Use `model_selection=0` for close-range (faster)
- Reduce camera resolution if needed: `width=320,height=240`
- Set `drop=true` in appsink to skip frames if processing is slow
- For production, compile to C++ using MediaPipe's C++ API

## Next Steps

Once working, you can:
1. Publish face coordinates as ROS topics
2. Control robot based on face position
3. Count faces for people detection
4. Face recognition by adding face embeddings

