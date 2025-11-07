#!/usr/bin/env python3
"""
Sparse Optical Flow (Lucas-Kanade) - MUCH FASTER than dense
Tracks feature points instead of every pixel

Architecture:
  GStreamer v4l2src (camera) → OpenCV Sparse Optical Flow → ROS publish
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


class GstSparseOpticalFlowNode(Node):
    def __init__(self):
        super().__init__('gst_sparse_optical_flow')
        
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Sparse optical flow parameters
        self.prev_gray = None
        self.prev_points = None
        
        # Parameters for ShiTomasi corner detection
        self.feature_params = dict(
            maxCorners=100,
            qualityLevel=0.3,
            minDistance=7,
            blockSize=7
        )
        
        # Parameters for Lucas-Kanade optical flow
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )
        
        # GStreamer pipeline
        self.pipeline_str = (
            'v4l2src device=/dev/video0 ! '
            'video/x-raw,width=640,height=480,framerate=30/1 ! '
            'videoconvert ! video/x-raw,format=RGB ! '
            'appsink name=sink emit-signals=true max-buffers=1 drop=true'
        )
        
        self.pipeline = Gst.parse_launch(self.pipeline_str)
        self.appsink = self.pipeline.get_by_name('sink')
        self.appsink.connect('new-sample', self.on_new_sample)
        
        # Start GStreamer
        self.gst_thread = threading.Thread(target=self.run_gstreamer, daemon=True)
        self.gst_thread.start()
        
        self.get_logger().info('=== Sparse Optical Flow Started ===')
        self.get_logger().info('Algorithm: Lucas-Kanade (tracks feature points)')
        self.get_logger().info('Much faster than Dense Optical Flow')
        self.get_logger().info('View at: http://ROBOT_IP:8080')
        
    def on_new_sample(self, sink):
        """Process each frame with sparse optical flow"""
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
        
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        
        # Detect new feature points if needed
        if self.prev_points is None or len(self.prev_points) < 10:
            self.prev_points = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
        
        output = frame_bgr.copy()
        
        # Calculate optical flow if we have previous frame and points
        if self.prev_gray is not None and self.prev_points is not None:
            # Calculate optical flow (Lucas-Kanade)
            new_points, status, error = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, gray, self.prev_points, None, **self.lk_params
            )
            
            if new_points is not None:
                # Select good points
                good_new = new_points[status == 1]
                good_old = self.prev_points[status == 1]
                
                # Draw motion vectors
                for i, (new, old) in enumerate(zip(good_new, good_old)):
                    a, b = new.ravel()
                    c, d = old.ravel()
                    a, b, c, d = int(a), int(b), int(c), int(d)
                    
                    # Draw line showing motion
                    cv2.line(output, (c, d), (a, b), (0, 255, 0), 2)
                    # Draw current position
                    cv2.circle(output, (a, b), 3, (0, 0, 255), -1)
                
                # Update points for next frame
                self.prev_points = good_new.reshape(-1, 1, 2)
        
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
    node = GstSparseOpticalFlowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

