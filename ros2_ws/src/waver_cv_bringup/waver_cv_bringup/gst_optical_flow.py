#!/usr/bin/env python3
"""
Optical Flow Motion Detection with GStreamer camera capture
Shows moving particles/elements in the video

Architecture:
  GStreamer v4l2src (camera) → OpenCV Optical Flow → ROS publish
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import numpy as np
import cv2
import threading

Gst.init(None)


class GstOpticalFlowNode(Node):
    def __init__(self):
        super().__init__('gst_optical_flow')
        
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Optical flow parameters
        self.prev_gray = None
        self.hsv = None
        
        # GStreamer pipeline - capture from USB webcam
        # Lower resolution = much faster optical flow
        self.pipeline_str = (
            'v4l2src device=/dev/video0 ! '
            'video/x-raw,width=320,height=240,framerate=30/1 ! '  # Reduced 4x pixels
            'videoconvert ! video/x-raw,format=RGB ! '
            'appsink name=sink emit-signals=true max-buffers=1 drop=true'
        )
        
        self.pipeline = Gst.parse_launch(self.pipeline_str)
        self.appsink = self.pipeline.get_by_name('sink')
        self.appsink.connect('new-sample', self.on_new_sample)
        
        # Start GStreamer in separate thread
        self.gst_thread = threading.Thread(target=self.run_gstreamer, daemon=True)
        self.gst_thread.start()
        
        self.get_logger().info('=== Optical Flow Motion Detection Started ===')
        self.get_logger().info('Camera: USB Webcam /dev/video0')
        self.get_logger().info('Algorithm: Dense Optical Flow (Farneback)')
        self.get_logger().info('Publishing to: /camera/image_raw')
        self.get_logger().info('View at: http://ROBOT_IP:8080')
        
    def on_new_sample(self, sink):
        """Process each frame with optical flow"""
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
        
        # Convert to BGR and grayscale
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        
        # Initialize HSV image for flow visualization on first frame
        if self.hsv is None:
            self.hsv = np.zeros_like(frame_bgr)
            self.hsv[..., 1] = 255  # Full saturation
        
        # Calculate optical flow if we have a previous frame
        if self.prev_gray is not None:
            # Dense optical flow (Farneback method) - Optimized for speed
            flow = cv2.calcOpticalFlowFarneback(
                self.prev_gray, gray,
                None,
                pyr_scale=0.5,
                levels=2,  # Reduced from 3 (faster)
                winsize=10,  # Reduced from 15 (faster)
                iterations=2,  # Reduced from 3 (faster)
                poly_n=5,
                poly_sigma=1.1,  # Reduced slightly
                flags=0
            )
            
            # Convert flow to polar coordinates (magnitude and angle)
            magnitude, angle = cv2.cartToPolar(flow[..., 0], flow[..., 1])
            
            # Encode flow direction as hue, magnitude as value
            self.hsv[..., 0] = angle * 180 / np.pi / 2  # Hue = direction
            self.hsv[..., 2] = cv2.normalize(magnitude, None, 0, 255, cv2.NORM_MINMAX)  # Value = speed
            
            # Convert HSV to BGR for visualization
            flow_bgr = cv2.cvtColor(self.hsv, cv2.COLOR_HSV2BGR)
            
            # Option 1: Show only flow (motion visualization)
            output = flow_bgr
            
            # Option 2: Overlay flow on original image
            # output = cv2.addWeighted(frame_bgr, 0.5, flow_bgr, 0.5, 0)
            
            # Option 3: Show motion mask (binary: moving vs static)
            # motion_mask = magnitude > 2  # Threshold for "moving"
            # output = frame_bgr.copy()
            # output[motion_mask] = [0, 255, 0]  # Green for moving areas
        else:
            # First frame - just show original
            output = frame_bgr
        
        # Update previous frame
        self.prev_gray = gray.copy()
        
        # Publish to ROS
        try:
            ros_image = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
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
    node = GstOpticalFlowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

