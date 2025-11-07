#!/usr/bin/env python3
"""
MediaPipe Face Detection with GStreamer camera capture
Publishes to ROS topics

Architecture:
  GStreamer v4l2src (camera) → Python MediaPipe (face detection) → ROS publish
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import mediapipe as mp
import numpy as np
import cv2
import threading

Gst.init(None)


class GstMediaPipeFaceNode(Node):
    def __init__(self):
        super().__init__('gst_mediapipe_face')
        
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Initialize MediaPipe Face Detection
        self.mp_face_detection = mp.solutions.face_detection
        self.mp_drawing = mp.solutions.drawing_utils
        self.face_detection = self.mp_face_detection.FaceDetection(
            model_selection=0,  # 0 = short-range (<2m), 1 = full-range
            min_detection_confidence=0.5
        )
        
        # GStreamer pipeline - capture from USB webcam
        self.pipeline_str = (
            'v4l2src device=/dev/video0 ! '
            'video/x-raw,width=640,height=480,framerate=30/1 ! '
            'videoconvert ! video/x-raw,format=RGB ! '
            'appsink name=sink emit-signals=true max-buffers=1 drop=true'
        )
        
        self.pipeline = Gst.parse_launch(self.pipeline_str)
        self.appsink = self.pipeline.get_by_name('sink')
        self.appsink.connect('new-sample', self.on_new_sample)
        
        # Start GStreamer in separate thread
        self.gst_thread = threading.Thread(target=self.run_gstreamer, daemon=True)
        self.gst_thread.start()
        
        self.get_logger().info('=== MediaPipe Face Detection Started ===')
        self.get_logger().info('Camera: USB Webcam /dev/video0')
        self.get_logger().info('Publishing to: /camera/image_raw')
        self.get_logger().info('View at: http://ROBOT_IP:8080')
        
    def on_new_sample(self, sink):
        """Process each frame from camera with MediaPipe"""
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
        frame_rgb = np.ndarray(
            shape=(height, width, 3),
            dtype=np.uint8,
            buffer=map_info.data
        ).copy()
        
        # Process with MediaPipe
        results = self.face_detection.process(frame_rgb)
        
        # Convert to BGR for OpenCV drawing
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        
        # Draw face detections
        if results.detections:
            for detection in results.detections:
                bbox = detection.location_data.relative_bounding_box
                x = int(bbox.xmin * width)
                y = int(bbox.ymin * height)
                w = int(bbox.width * width)
                h = int(bbox.height * height)
                
                # Draw green rectangle
                cv2.rectangle(frame_bgr, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                # Draw confidence score
                score = detection.score[0]
                cv2.putText(
                    frame_bgr,
                    f'Face: {score:.2f}',
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )
                
                # Draw facial keypoints (eyes, nose, mouth, ears)
                for keypoint in detection.location_data.relative_keypoints:
                    kp_x = int(keypoint.x * width)
                    kp_y = int(keypoint.y * height)
                    cv2.circle(frame_bgr, (kp_x, kp_y), 3, (0, 0, 255), -1)
        
        # Publish to ROS
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame_bgr, encoding='bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'usb_webcam_frame'
            self.publisher.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Failed to publish: {e}')
        
        buffer.unmap(map_info)
        return Gst.FlowReturn.OK
        
    def run_gstreamer(self):
        """Run GStreamer pipeline"""
        self.pipeline.set_state(Gst.State.PLAYING)
        self.loop = GLib.MainLoop()
        try:
            self.loop.run()
        except Exception as e:
            self.get_logger().error(f'GStreamer error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GstMediaPipeFaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

