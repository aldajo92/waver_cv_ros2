#!/usr/bin/env python3
"""
GStreamer node that splits camera stream using tee:
  - Raw video branch → /camera/image_raw
  - Optical flow branch → /camera/optical_flow

Architecture:
  v4l2src → videoconvert → tee
    → branch 1: queue → appsink (raw)
    → branch 2: queue → opencvoptflow → appsink (optical flow)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import numpy as np
import cv2
import threading

Gst.init(None)


class GstSplitRawOptFlowNode(Node):
    def __init__(self):
        super().__init__('gst_split_raw_optflow')
        
        # Publishers for both streams
        self.raw_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.optflow_publisher = self.create_publisher(Image, '/camera/optical_flow', 10)
        self.raw_camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.optflow_camera_info_pub = self.create_publisher(CameraInfo, '/camera/optical_flow_camera_info', 10)
        
        self.bridge = CvBridge()
        
        # GStreamer pipeline with tee to split stream
        # v4l2src → videoconvert → tee
        #   → branch 1: raw video → appsink
        #   → branch 2: optical flow → appsink
        self.pipeline_str = (
            'v4l2src device=/dev/video0 ! '
            'video/x-raw,width=320,height=240,framerate=30/1 ! '
            'videoconvert ! video/x-raw,format=RGB ! '
            'tee name=t ! '
            'queue ! appsink name=raw_sink emit-signals=true max-buffers=1 drop=true '
            't. ! queue ! opencvoptflow ! videoconvert ! video/x-raw,format=RGB ! '
            'appsink name=optflow_sink emit-signals=true max-buffers=1 drop=true'
        )
        
        self.pipeline = Gst.parse_launch(self.pipeline_str)
        
        # Get both appsinks
        self.raw_appsink = self.pipeline.get_by_name('raw_sink')
        self.optflow_appsink = self.pipeline.get_by_name('optflow_sink')
        
        # Connect callbacks
        self.raw_appsink.connect('new-sample', self.on_raw_sample)
        self.optflow_appsink.connect('new-sample', self.on_optflow_sample)
        
        # Camera info (same for both streams)
        self.camera_info = CameraInfo()
        self.camera_info.width = 320
        self.camera_info.height = 240
        self.camera_info.distortion_model = 'plumb_bob'
        
        # Start GStreamer in separate thread
        self.gst_thread = threading.Thread(target=self.run_gstreamer, daemon=True)
        self.gst_thread.start()
        
        self.get_logger().info('GStreamer split node started: raw → /camera/image_raw, optical flow → /camera/optical_flow')
    
    def on_raw_sample(self, appsink):
        """Handle raw video frame"""
        sample = appsink.emit('pull-sample')
        if sample:
            buffer = sample.get_buffer()
            caps = sample.get_caps()
            
            # Get frame info
            structure = caps.get_structure(0)
            width = structure.get_int('width').value
            height = structure.get_int('height').value
            
            # Extract frame data
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if not success:
                return Gst.FlowReturn.OK
            
            try:
                # Create numpy array from buffer
                frame_data = np.frombuffer(map_info.data, dtype=np.uint8)
                frame = frame_data.reshape((height, width, 3))
                
                # Convert to ROS Image message
                ros_image = self.bridge.cv2_to_imgmsg(frame, 'rgb8')
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = 'usb_webcam_frame'
                
                # Publish raw image
                self.raw_publisher.publish(ros_image)
                
                # Publish camera info
                self.camera_info.header = ros_image.header
                self.raw_camera_info_pub.publish(self.camera_info)
                
            finally:
                buffer.unmap(map_info)
        
        return Gst.FlowReturn.OK
    
    def on_optflow_sample(self, appsink):
        """Handle optical flow processed frame"""
        sample = appsink.emit('pull-sample')
        if sample:
            buffer = sample.get_buffer()
            caps = sample.get_caps()
            
            # Get frame info
            structure = caps.get_structure(0)
            width = structure.get_int('width').value
            height = structure.get_int('height').value
            
            # Extract frame data
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if not success:
                return Gst.FlowReturn.OK
            
            try:
                # Create numpy array from buffer
                frame_data = np.frombuffer(map_info.data, dtype=np.uint8)
                frame = frame_data.reshape((height, width, 3))
                
                # Convert to ROS Image message
                ros_image = self.bridge.cv2_to_imgmsg(frame, 'rgb8')
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = 'usb_webcam_frame'
                
                # Publish optical flow image
                self.optflow_publisher.publish(ros_image)
                
                # Publish camera info
                self.camera_info.header = ros_image.header
                self.optflow_camera_info_pub.publish(self.camera_info)
                
            finally:
                buffer.unmap(map_info)
        
        return Gst.FlowReturn.OK
    
    def run_gstreamer(self):
        """Run GStreamer pipeline in separate thread"""
        # Set pipeline to playing state
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error('Failed to start GStreamer pipeline')
            return
        
        # Run GLib main loop
        loop = GLib.MainLoop()
        loop.run()


def main(args=None):
    rclpy.init(args=args)
    node = GstSplitRawOptFlowNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.set_state(Gst.State.NULL)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

