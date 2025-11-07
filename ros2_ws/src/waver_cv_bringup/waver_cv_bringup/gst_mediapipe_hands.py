#!/usr/bin/env python3
"""
MediaPipe Hand Tracking with GStreamer camera capture
Publishes to ROS topics

Architecture:
  GStreamer v4l2src (camera) → Python MediaPipe (hand tracking) → ROS publish
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


class GstMediaPipeHandsNode(Node):
    def __init__(self):
        super().__init__('gst_mediapipe_hands')
        
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,  # Track only 1 hand instead of 2 (2x faster)
            min_detection_confidence=0.7,  # Higher = less false positives, faster
            min_tracking_confidence=0.5,
            model_complexity=0  # 0 = lite model (faster), 1 = full model (accurate)
        )
        
        # GStreamer pipeline - capture from USB webcam
        # Lower resolution = faster processing
        self.pipeline_str = (
            'v4l2src device=/dev/video0 ! '
            'video/x-raw,width=320,height=240,framerate=30/1 ! '  # Reduced resolution
            'videoconvert ! video/x-raw,format=RGB ! '
            'appsink name=sink emit-signals=true max-buffers=1 drop=true'
        )
        
        self.pipeline = Gst.parse_launch(self.pipeline_str)
        self.appsink = self.pipeline.get_by_name('sink')
        self.appsink.connect('new-sample', self.on_new_sample)
        
        # Start GStreamer in separate thread
        self.gst_thread = threading.Thread(target=self.run_gstreamer, daemon=True)
        self.gst_thread.start()
        
        self.get_logger().info('=== MediaPipe Hand Tracking Started ===')
        self.get_logger().info('Camera: USB Webcam /dev/video0')
        self.get_logger().info('Max hands: 2')
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
        
        # Process with MediaPipe Hands
        results = self.hands.process(frame_rgb)
        
        # Convert to BGR for OpenCV drawing
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        
        # Draw hand landmarks
        if results.multi_hand_landmarks:
            for hand_idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                # Draw hand connections
                self.mp_drawing.draw_landmarks(
                    frame_bgr,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )
                
                # Get handedness (left/right)
                if results.multi_handedness:
                    handedness = results.multi_handedness[hand_idx]
                    hand_label = handedness.classification[0].label  # "Left" or "Right"
                    hand_score = handedness.classification[0].score
                    
                    # Draw hand label
                    # Get wrist position for text placement
                    wrist = hand_landmarks.landmark[0]
                    x = int(wrist.x * width)
                    y = int(wrist.y * height)
                    
                    cv2.putText(
                        frame_bgr,
                        f'{hand_label}: {hand_score:.2f}',
                        (x - 50, y - 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )
        
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
    node = GstMediaPipeHandsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

