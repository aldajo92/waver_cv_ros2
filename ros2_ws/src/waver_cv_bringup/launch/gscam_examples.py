"""
GStreamer Processing Examples for gscam2
Video is processed in GStreamer pipeline BEFORE being published to ROS

Usage: Copy the gscam_config you want to main_gscam.launch.py
"""

# ============================================================================
# EXAMPLE 1: Edge Detection (Built-in GStreamer)
# ============================================================================
gscam_edge_detection = (
    'v4l2src device=/dev/video0 ! '
    'video/x-raw,width=640,height=480 ! '
    'videoconvert ! '
    'edgetv ! '  # Edge detection effect
    'videoconvert ! video/x-raw,format=RGB'
)

# ============================================================================
# EXAMPLE 2: Grayscale Conversion
# ============================================================================
gscam_grayscale = (
    'v4l2src device=/dev/video0 ! '
    'video/x-raw,width=640,height=480 ! '
    'videoconvert ! '
    'video/x-raw,format=GRAY8 ! '  # Convert to grayscale
    'videoconvert ! video/x-raw,format=RGB'
)

# ============================================================================
# EXAMPLE 3: Color Balance / Brightness / Contrast
# ============================================================================
gscam_color_adjust = (
    'v4l2src device=/dev/video0 ! '
    'video/x-raw,width=640,height=480 ! '
    'videoconvert ! '
    'videobalance brightness=0.2 contrast=1.5 saturation=0.5 ! '  # Adjust colors
    'videoconvert ! video/x-raw,format=RGB'
)

# ============================================================================
# EXAMPLE 4: Blur Effect
# ============================================================================
gscam_blur = (
    'v4l2src device=/dev/video0 ! '
    'video/x-raw,width=640,height=480 ! '
    'videoconvert ! '
    'frei0r-filter-iirblur blur-level=0.5 ! '  # Blur filter
    'videoconvert ! video/x-raw,format=RGB'
)

# ============================================================================
# EXAMPLE 5: Motion Detection (shows only moving parts)
# ============================================================================
gscam_motion = (
    'v4l2src device=/dev/video0 ! '
    'video/x-raw,width=640,height=480 ! '
    'videoconvert ! '
    'videodetect ! '  # Motion detection
    'videoconvert ! video/x-raw,format=RGB'
)

# ============================================================================
# EXAMPLE 6: Multiple Effects Combined
# ============================================================================
gscam_combined = (
    'v4l2src device=/dev/video0 ! '
    'video/x-raw,width=640,height=480 ! '
    'videoconvert ! '
    'videobalance saturation=0.0 ! '  # Grayscale via desaturation
    'edgetv ! '  # Edge detection
    'gamma gamma=1.5 ! '  # Increase gamma
    'videoconvert ! video/x-raw,format=RGB'
)

# ============================================================================
# EXAMPLE 7: Face Detection (if opencv gstreamer plugin available)
# ============================================================================
gscam_face_detect = (
    'v4l2src device=/dev/video0 ! '
    'video/x-raw,width=640,height=480 ! '
    'videoconvert ! '
    'facedetect display=true ! '  # Draw boxes around faces
    'videoconvert ! video/x-raw,format=RGB'
)

# ============================================================================
# EXAMPLE 8: Sharpen Filter
# ============================================================================
gscam_sharpen = (
    'v4l2src device=/dev/video0 ! '
    'video/x-raw,width=640,height=480 ! '
    'videoconvert ! '
    'frei0r-filter-sharpen amount=0.8 size=0.3 ! '
    'videoconvert ! video/x-raw,format=RGB'
)

# ============================================================================
# EXAMPLE 9: Sepia Tone Effect
# ============================================================================
gscam_sepia = (
    'v4l2src device=/dev/video0 ! '
    'video/x-raw,width=640,height=480 ! '
    'videoconvert ! '
    'frei0r-filter-sepia intensity=0.8 ! '
    'videoconvert ! video/x-raw,format=RGB'
)

# ============================================================================
# EXAMPLE 10: Custom OpenCV Processing with gst-python (Advanced)
# Requires: python3-gst-1.0 package
# ============================================================================
# This would require a separate Python script that uses appsink/appsrc:
# 
# Camera → appsink (get frames) → Python OpenCV → appsrc (send back) → ROS
#
# gscam_python_opencv = (
#     'v4l2src device=/dev/video0 ! '
#     'video/x-raw,width=640,height=480 ! '
#     'videoconvert ! '
#     'appsink name=sink emit-signals=true ! '  # Extract frames to Python
#     'appsrc name=src ! '  # Inject processed frames back
#     'videoconvert ! video/x-raw,format=RGB'
# )
#
# Then in Python script, connect to appsink/appsrc signals
# See: /ros2_ws/src/waver_cv_bringup/scripts/gst_python_processor.py

# ============================================================================
# How to check available GStreamer plugins in your Docker container:
# ============================================================================
# gst-inspect-1.0 | grep -i filter
# gst-inspect-1.0 videobalance
# gst-inspect-1.0 edgetv
# gst-inspect-1.0 facedetect

