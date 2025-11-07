from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # GStreamer + MediaPipe face detection node
        # Uses GStreamer v4l2src for camera capture, MediaPipe for face detection
        Node(
            package='waver_cv_bringup',
            executable='gst_mediapipe_face',
            name='mediapipe_face_detection',
            output='screen',
        ),
        # Web video server
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server'
        )
    ])

