from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Camera configuration
    camera_name = 'rpi_camera'
    camera_frame = 'rpi_camera_frame'
    
    # GStreamer pipeline for Raspberry Pi camera using libcamera
    # This uses libcamerasrc which works with the libcamera stack you built in Docker
    gscam_config = 'libcamerasrc ! video/x-raw,width=640,height=480,format=BGR ! videoconvert'
    
    return LaunchDescription([
        # Node for gscam2 with Raspberry Pi camera
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
                    'camera_info_url': '',  # Empty for no calibration file
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

