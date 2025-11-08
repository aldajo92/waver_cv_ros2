# MediaPipe with GStreamer Integration

MediaPipe integrates directly into GStreamer pipelines - perfect for your use case!

## Installation (Inside Docker)

```bash
./scripts/shell.sh

# Inside container
pip3 install mediapipe opencv-python
```

## Option A: MediaPipe Python in GStreamer Pipeline

Create a custom GStreamer element that uses MediaPipe:

```python
# File: ros2_ws/src/waver_cv_bringup/scripts/gst_mediapipe_plugin.py

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import mediapipe as mp
import numpy as np
import cv2

Gst.init(None)

class MediaPipeProcessor:
    def __init__(self, solution='hands'):
        """
        solution: 'hands', 'pose', 'face_mesh', 'face_detection', 'holistic'
        """
        self.mp_drawing = mp.solutions.drawing_utils
        
        if solution == 'hands':
            self.mp_solution = mp.solutions.hands
            self.detector = self.mp_solution.Hands(
                static_image_mode=False,
                max_num_hands=2,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5
            )
        elif solution == 'pose':
            self.mp_solution = mp.solutions.pose
            self.detector = self.mp_solution.Pose(
                static_image_mode=False,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5
            )
        elif solution == 'face_detection':
            self.mp_solution = mp.solutions.face_detection
            self.detector = self.mp_solution.FaceDetection(
                min_detection_confidence=0.5
            )
        
        # GStreamer pipeline
        self.pipeline_str = (
            'v4l2src device=/dev/video0 ! '
            'video/x-raw,width=640,height=480,format=RGB ! '
            'videoconvert ! '
            'appsink name=sink emit-signals=true max-buffers=1 drop=true'
        )
        
        self.pipeline = Gst.parse_launch(self.pipeline_str)
        self.appsink = self.pipeline.get_by_name('sink')
        self.appsink.connect('new-sample', self.on_new_sample)
        
    def on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.ERROR
            
        buffer = sample.get_buffer()
        caps = sample.get_caps()
        
        structure = caps.get_structure(0)
        width = structure.get_value('width')
        height = structure.get_value('height')
        
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR
        
        # Convert to numpy
        frame = np.ndarray(
            shape=(height, width, 3),
            dtype=np.uint8,
            buffer=map_info.data
        )
        
        # Process with MediaPipe
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.detector.process(frame_rgb)
        
        # Draw results
        if hasattr(results, 'multi_hand_landmarks') and results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame, hand_landmarks, self.mp_solution.HAND_CONNECTIONS
                )
        elif hasattr(results, 'pose_landmarks') and results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                frame, results.pose_landmarks, self.mp_solution.POSE_CONNECTIONS
            )
        elif hasattr(results, 'detections') and results.detections:
            for detection in results.detections:
                self.mp_drawing.draw_detection(frame, detection)
        
        # Here you can push back to another appsrc to send to ROS
        # For now, just display
        cv2.imshow('MediaPipe', frame)
        cv2.waitKey(1)
        
        buffer.unmap(map_info)
        return Gst.FlowReturn.OK
        
    def run(self):
        self.pipeline.set_state(Gst.State.PLAYING)
        loop = GLib.MainLoop()
        try:
            loop.run()
        except KeyboardInterrupt:
            pass
        self.pipeline.set_state(Gst.State.NULL)
```

## Option B: MediaPipe C++ GStreamer Plugin (Most Efficient)

MediaPipe provides C++ calculators that can be compiled into GStreamer plugins.

### 1. Add MediaPipe to Dockerfile

```dockerfile
# Install MediaPipe dependencies
RUN apt update && apt install -y \
    build-essential \
    libopencv-dev \
    cmake \
    protobuf-compiler

# Clone MediaPipe
RUN git clone https://github.com/google/mediapipe.git /mediapipe
WORKDIR /mediapipe

# Build MediaPipe with GStreamer support
RUN bazel build -c opt --define MEDIAPIPE_DISABLE_GPU=1 \
    mediapipe/examples/desktop:hand_tracking_cpu
```

### 2. Use MediaPipe Calculator in GStreamer

```cpp
// Custom GStreamer element using MediaPipe
// See: /gst_custom_plugin/gst_mediapipe_filter.cpp

#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include <gst/gst.h>

// MediaPipe graph config
const char* kGraphConfig = R"(
    input_stream: "input_video"
    output_stream: "output_video"
    
    node {
        calculator: "HandLandmarkTrackingCpu"
        input_stream: "IMAGE:input_video"
        output_stream: "LANDMARKS:hand_landmarks"
        output_stream: "NORM_LANDMARKS:hand_norm_landmarks"
    }
    
    node {
        calculator: "HandRenderer"
        input_stream: "IMAGE:input_video"
        input_stream: "LANDMARKS:hand_landmarks"
        output_stream: "IMAGE:output_video"
    }
)";
```

## Recommendation

**Start with Python MediaPipe + appsink/appsrc** (Option A):
- ✅ Easier to develop and iterate
- ✅ Full MediaPipe solutions available
- ✅ Good performance for most use cases
- ❌ Slightly higher CPU usage than C++

**Move to C++ MediaPipe if needed** (Option B):
- ✅ Maximum performance
- ✅ Lower latency
- ❌ More complex build process
- ❌ Requires Bazel and longer compile times

## Quick Test

```bash
pip3 install mediapipe opencv-python
python3 ros2_ws/src/waver_cv_bringup/scripts/gst_mediapipe_plugin.py
```

Would you like me to create the complete MediaPipe integration?

