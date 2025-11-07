from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # GStreamer + Sparse Optical Flow (Lucas-Kanade) - FASTER
        Node(
            package='waver_cv_bringup',
            executable='gst_sparse_optical_flow',
            name='sparse_optical_flow',
            output='screen',
        ),
        # Web video server
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server'
        )
    ])

