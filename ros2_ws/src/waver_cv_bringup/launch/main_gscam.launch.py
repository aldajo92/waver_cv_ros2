from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Camera configuration
    camera_name = 'usb_webcam'
    camera_frame = 'usb_webcam_frame'
    
    # GStreamer pipeline for USB webcam (Logitech C920)
    # Uses v4l2src to access camera via /dev/video0
    # gscam_config = 'v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480 ! videoconvert ! video/x-raw,format=RGB'
    gscam_config = 'v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480 ! videoconvert ! edgetv ! videoconvert ! video/x-raw,format=RGB'
    
    
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
                ('image_raw', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
            ],
        ),
        # Node for the web video server
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server'
        )
    ])

