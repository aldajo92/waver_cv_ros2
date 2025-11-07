from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # GStreamer + Optical Flow motion detection
        # Uses GStreamer v4l2src for camera, OpenCV for optical flow
        Node(
            package='waver_cv_bringup',
            executable='gst_optical_flow',
            name='optical_flow_detection',
            output='screen',
        ),
        # Web video server
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server'
        )
    ])

