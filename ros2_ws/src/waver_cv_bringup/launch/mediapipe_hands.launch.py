from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # GStreamer + MediaPipe hand tracking node
        # Uses GStreamer v4l2src for camera capture, MediaPipe for hand tracking
        Node(
            package='waver_cv_bringup',
            executable='gst_mediapipe_hands',
            name='mediapipe_hands_tracking',
            output='screen',
        ),
        # Web video server
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server'
        )
    ])

