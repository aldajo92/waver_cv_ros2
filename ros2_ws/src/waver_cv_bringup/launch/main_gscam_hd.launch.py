from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments for customization
    width_arg = DeclareLaunchArgument('width', default_value='1280', description='Camera width')
    height_arg = DeclareLaunchArgument('height', default_value='720', description='Camera height')
    framerate_arg = DeclareLaunchArgument('framerate', default_value='30', description='Camera framerate')
    
    # Get launch configuration values
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    framerate = LaunchConfiguration('framerate')
    
    # Camera configuration
    camera_name = 'rpi_camera'
    camera_frame = 'rpi_camera_frame'
    
    # GStreamer pipeline for Raspberry Pi camera using libcamera
    # Format: libcamerasrc ! video/x-raw,width=W,height=H,framerate=F/1,format=BGR ! videoconvert
    # Note: LaunchConfiguration doesn't support f-strings directly, so we use a fixed HD resolution
    gscam_config = 'libcamerasrc ! video/x-raw,width=1280,height=720,framerate=30/1,format=BGR ! videoconvert'
    
    return LaunchDescription([
        width_arg,
        height_arg,
        framerate_arg,
        
        # Node for gscam2 with Raspberry Pi camera (HD resolution)
        Node(
            package='gscam2',
            executable='gscam_main',
            output='screen',
            name='gscam_publisher',
            namespace=camera_name,
            parameters=[
                {
                    'gscam_config': gscam_config,
                    'camera_name': camera_name,
                    'frame_id': camera_frame,
                    'preroll': False,
                    'use_gst_timestamps': False,
                    'camera_info_url': '',
                }
            ],
            remappings=[
                ('image_raw', '/image_raw'),
                ('camera_info', '/camera_info'),
            ],
        ),
        # Node for the simple subscriber and publisher using OpenCV
        Node(
            package='waver_cv',
            executable='simple_sub_pub',
            name='simple_sub_pub_node',
        ),
        # Node for the web video server
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server'
        )
    ])

