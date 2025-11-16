from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file that splits camera stream using GStreamer tee:
      - Raw video → /camera/image_raw
      - Optical flow → /camera/optical_flow
    
    Pipeline architecture:
      v4l2src → videoconvert → tee
        → branch 1: queue → appsink (raw video)
        → branch 2: queue → opencvoptflow → appsink (optical flow)
    """
    
    return LaunchDescription([
        # Node that splits camera stream using GStreamer tee
        # Publishes both raw and optical flow streams
        Node(
            package='waver_cv_bringup',
            executable='gst_split_raw_optflow',
            name='gst_split_raw_optflow',
            output='screen',
        ),
        # Node for the web video server
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server'
        )
    ])

