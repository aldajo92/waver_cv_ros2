from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    stream_port_arg = DeclareLaunchArgument(
        'stream_port',
        default_value='5000',
        description='UDP port for GStreamer streaming'
    )
    
    stream_encoding_arg = DeclareLaunchArgument(
        'stream_encoding',
        default_value='h264',
        description='Encoding type: h264, mjpeg, or raw'
    )
    
    stream_bitrate_arg = DeclareLaunchArgument(
        'stream_bitrate',
        default_value='4000000',
        description='Bitrate for H264 encoding (bits per second) - higher for HD'
    )
    
    # Get launch configuration values
    stream_port = LaunchConfiguration('stream_port')
    stream_encoding = LaunchConfiguration('stream_encoding')
    stream_bitrate = LaunchConfiguration('stream_bitrate')
    
    # Camera configuration
    camera_name = 'rpi_camera'
    camera_frame = 'rpi_camera_frame'
    
    # GStreamer pipeline for Raspberry Pi camera using libcamera - HD resolution
    gscam_config = 'libcamerasrc ! video/x-raw,width=1280,height=720,framerate=30/1,format=BGR ! videoconvert'
    
    return LaunchDescription([
        stream_port_arg,
        stream_encoding_arg,
        stream_bitrate_arg,
        
        # Node for gscam2 with Raspberry Pi camera (HD)
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
        
        # Node for GStreamer streaming (view without ROS on remote computer) - HD
        Node(
            package='waver_cv',
            executable='gstreamer_streamer',
            output='screen',
            name='gstreamer_streamer',
            parameters=[
                {
                    'input_topic': '/image_raw',
                    'host': '0.0.0.0',
                    'port': stream_port,
                    'encoding': stream_encoding,
                    'bitrate': stream_bitrate,
                    'width': 1280,
                    'height': 720,
                }
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

