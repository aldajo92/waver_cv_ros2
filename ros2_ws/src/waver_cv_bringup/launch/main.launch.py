from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Node for the web camera: ros2 launch camera_ros camera.launch.py width:=640 height:=480 format:=BGR888
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('camera_ros'),
                    'launch',
                    'camera.launch.py'
                ])
            ),
            launch_arguments={
                'width': '640',
                'height': '480',
                'format': 'BGR888'
            }.items()
        ),  
        # Node for the simple subscriber and publisher using OpenCV
        Node(
            package='waver_cv',
            executable='simple_sub_pub',
            name='simple_sub_pub_node',
        ),
        # Node for the web camera server
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server'
        )
    ])
