#!/usr/bin/env python3
"""
GStreamer MediaPipe Face Detection Element
This is a GStreamer PLUGIN, not a ROS node

Install in container:
  cp gst_mediapipe_face.py /usr/local/lib/gstreamer-1.0/python/
  
Use in pipeline:
  gst-launch-1.0 v4l2src ! videoconvert ! mediapipe_face ! videoconvert ! autovideosink
"""

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstBase', '1.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import Gst, GObject, GstBase, GstVideo
import mediapipe as mp
import numpy as np
import cv2

Gst.init(None)


class MediaPipeFace(GstBase.BaseTransform):
    """
    GStreamer element that applies MediaPipe face detection to video frames
    """
    
    __gstmetadata__ = (
        "MediaPipe Face Detection",
        "Filter/Effect/Video",
        "Detect faces using MediaPipe",
        "Your Name"
    )

    __gsttemplates__ = (
        Gst.PadTemplate.new(
            "src",
            Gst.PadDirection.SRC,
            Gst.PadPresence.ALWAYS,
            Gst.Caps.from_string("video/x-raw,format=RGB")
        ),
        Gst.PadTemplate.new(
            "sink",
            Gst.PadDirection.SINK,
            Gst.PadPresence.ALWAYS,
            Gst.Caps.from_string("video/x-raw,format=RGB")
        )
    )

    def __init__(self):
        super(MediaPipeFace, self).__init__()
        
        # Initialize MediaPipe Face Detection
        self.mp_face_detection = mp.solutions.face_detection
        self.face_detection = self.mp_face_detection.FaceDetection(
            model_selection=0,
            min_detection_confidence=0.5
        )
        
        self.set_in_place(True)
        
    def do_transform_ip(self, buf):
        """Process frame in-place"""
        try:
            # Get video info
            caps = self.sinkpad.get_current_caps()
            if not caps:
                return Gst.FlowReturn.ERROR
                
            structure = caps.get_structure(0)
            width = structure.get_int('width')[1]
            height = structure.get_int('height')[1]
            
            # Map buffer
            success, mapinfo = buf.map(Gst.MapFlags.READ | Gst.MapFlags.WRITE)
            if not success:
                return Gst.FlowReturn.ERROR
            
            # Convert to numpy array
            frame = np.ndarray(
                shape=(height, width, 3),
                dtype=np.uint8,
                buffer=mapinfo.data
            )
            
            # Process with MediaPipe
            results = self.face_detection.process(frame)
            
            # Draw detections
            if results.detections:
                for detection in results.detections:
                    bbox = detection.location_data.relative_bounding_box
                    x = int(bbox.xmin * width)
                    y = int(bbox.ymin * height)
                    w = int(bbox.width * width)
                    h = int(bbox.height * height)
                    
                    # Draw rectangle
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    
                    # Draw score
                    score = detection.score[0]
                    cv2.putText(
                        frame,
                        f'Face: {score:.2f}',
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )
                    
                    # Draw keypoints
                    for keypoint in detection.location_data.relative_keypoints:
                        kp_x = int(keypoint.x * width)
                        kp_y = int(keypoint.y * height)
                        cv2.circle(frame, (kp_x, kp_y), 3, (0, 0, 255), -1)
            
            buf.unmap(mapinfo)
            return Gst.FlowReturn.OK
            
        except Exception as e:
            print(f"Error in MediaPipe: {e}")
            if 'mapinfo' in locals():
                buf.unmap(mapinfo)
            return Gst.FlowReturn.ERROR


# Register plugin
GObject.type_register(MediaPipeFace)
__gstelementfactory__ = (
    "mediapipe_face",
    Gst.Rank.NONE,
    MediaPipeFace
)

