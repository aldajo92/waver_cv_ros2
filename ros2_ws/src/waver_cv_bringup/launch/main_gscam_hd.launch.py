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
    camera_name = 'usb_webcam'
    camera_frame = 'usb_webcam_frame'
    
    # GStreamer pipeline for USB webcam (Logitech C920) - HD resolution
    # Uses v4l2src to access camera via /dev/video0
    # videoflip method=2 rotates 180 degrees
    # Note: LaunchConfiguration doesn't support f-strings directly, so we use a fixed HD resolution
    gscam_config = 'v4l2src device=/dev/video0 ! video/x-raw,width=1280,height=720,framerate=30/1 ! videoflip method=2 ! videoconvert ! video/x-raw,format=RGB'
    
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
                ('image_raw', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
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

