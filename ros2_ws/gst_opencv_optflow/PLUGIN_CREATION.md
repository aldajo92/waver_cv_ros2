# GStreamer C++ Plugin Creation Guide

## Overview

A GStreamer plugin is a shared library (`.so`) that adds new elements to GStreamer pipelines.

**Architecture:**
```
Your Plugin (.cpp) → Compile (.so) → Install to /usr/lib/.../gstreamer-1.0/ → Use in pipeline
```

---

## Plugin Structure

Every GStreamer plugin needs these components:

### 1. **Type Definition** (Lines 18-29 in `gst_opencv_optflow.cpp`)

```cpp
// Define your plugin type
#define GST_TYPE_OPENCV_OPTFLOW (gst_opencv_optflow_get_type())
#define GST_OPENCV_OPTFLOW(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_OPENCV_OPTFLOW,GstOpenCVOptFlow))

// Your plugin structure - add your custom data here
struct _GstOpenCVOptFlow {
    GstVideoFilter videofilter;    // Base class
    
    // Add your custom variables:
    cv::Mat prev_gray;             // Previous frame
    cv::Mat hsv;                   // HSV image for visualization
    gboolean initialized;          // Initialization flag
};
```

### 2. **Transform Function** (Lines 38-108)

This is where you process each frame:

```cpp
static GstFlowReturn
gst_opencv_optflow_transform_frame_ip(GstVideoFilter *filter, GstVideoFrame *frame)
{
    GstOpenCVOptFlow *optflow = GST_OPENCV_OPTFLOW(filter);
    
    // Get frame dimensions
    gint width = GST_VIDEO_FRAME_WIDTH(frame);
    gint height = GST_VIDEO_FRAME_HEIGHT(frame);
    guint8 *data = (guint8 *)GST_VIDEO_FRAME_PLANE_DATA(frame, 0);
    gint stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);
    
    // Create OpenCV Mat from frame buffer
    cv::Mat img(height, width, CV_8UC3, data, stride);
    
    // YOUR PROCESSING HERE
    // - img is the current frame (RGB format)
    // - Modify it directly (in-place processing)
    // - Access previous data via optflow->your_variable
    
    return GST_FLOW_OK;
}
```

### 3. **Class Initialization** (Lines 110-133)

Define plugin metadata and capabilities:

```cpp
static void
gst_opencv_optflow_class_init(GstOpenCVOptFlowClass *klass)
{
    GstElementClass *element_class = GST_ELEMENT_CLASS(klass);
    GstVideoFilterClass *vfilter_class = GST_VIDEO_FILTER_CLASS(klass);

    // Link your transform function
    vfilter_class->transform_frame_ip = gst_opencv_optflow_transform_frame_ip;

    // Plugin metadata (shows in gst-inspect-1.0)
    gst_element_class_set_static_metadata(element_class,
        "OpenCV Optical Flow",              // Long name
        "Filter/Effect/Video",              // Classification
        "Dense optical flow using OpenCV",  // Description
        "Your Name");                       // Author

    // Define what video formats your plugin accepts
    GstCaps *caps = gst_caps_from_string(GST_VIDEO_CAPS_MAKE("RGB"));
    // Or multiple formats: GST_VIDEO_CAPS_MAKE("{ RGB, BGR, GRAY8 }")
    
    gst_element_class_add_pad_template(element_class,
        gst_pad_template_new("src", GST_PAD_SRC, GST_PAD_ALWAYS, caps));
    gst_element_class_add_pad_template(element_class,
        gst_pad_template_new("sink", GST_PAD_SINK, GST_PAD_ALWAYS, caps));
    
    gst_caps_unref(caps);
}
```

### 4. **Instance Initialization** (Lines 135-139)

Initialize your variables:

```cpp
static void
gst_opencv_optflow_init(GstOpenCVOptFlow *filter)
{
    filter->initialized = FALSE;
    // Initialize other variables here
}
```

### 5. **Plugin Registration** (Lines 141-156)

Register your plugin with GStreamer:

```cpp
static gboolean
plugin_init(GstPlugin *plugin)
{
    GST_DEBUG_CATEGORY_INIT(gst_opencv_optflow_debug, "opencvoptflow", 0, "OpenCV optical flow");
    return gst_element_register(
        plugin,
        "opencvoptflow",           // Element name (use in pipeline)
        GST_RANK_NONE,
        GST_TYPE_OPENCV_OPTFLOW
    );
}

GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    opencvoptflow,                 // Plugin name
    "OpenCV optical flow filter",  // Description
    plugin_init,
    "1.0",
    "LGPL",
    "GStreamer",
    "http://gstreamer.net/"
)
```

---

## Creating a New Plugin

### Step 1: Create Directory and Files

```bash
mkdir ros2_ws/gst_my_plugin
cd ros2_ws/gst_my_plugin
```

### Step 2: Create `gst_my_filter.cpp`

```cpp
#define PACKAGE "myfilter"

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideofilter.h>
#include <opencv2/opencv.hpp>

GST_DEBUG_CATEGORY_STATIC(gst_my_filter_debug);
#define GST_CAT_DEFAULT gst_my_filter_debug

#define GST_TYPE_MY_FILTER (gst_my_filter_get_type())
#define GST_MY_FILTER(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_MY_FILTER,GstMyFilter))

typedef struct _GstMyFilter GstMyFilter;
typedef struct _GstMyFilterClass GstMyFilterClass;

struct _GstMyFilter {
    GstVideoFilter videofilter;
    // Add your variables
    gint threshold;
};

struct _GstMyFilterClass {
    GstVideoFilterClass parent_class;
};

GType gst_my_filter_get_type(void);

#define gst_my_filter_parent_class parent_class
G_DEFINE_TYPE(GstMyFilter, gst_my_filter, GST_TYPE_VIDEO_FILTER);

static GstFlowReturn
gst_my_filter_transform_frame_ip(GstVideoFilter *filter, GstVideoFrame *frame)
{
    GstMyFilter *myfilter = GST_MY_FILTER(filter);
    
    gint width = GST_VIDEO_FRAME_WIDTH(frame);
    gint height = GST_VIDEO_FRAME_HEIGHT(frame);
    guint8 *data = (guint8 *)GST_VIDEO_FRAME_PLANE_DATA(frame, 0);
    gint stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);
    
    // Create OpenCV Mat
    cv::Mat img(height, width, CV_8UC3, data, stride);
    
    // YOUR PROCESSING HERE
    // Example: Convert to grayscale and back
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
    cv::cvtColor(gray, img, cv::COLOR_GRAY2RGB);
    
    return GST_FLOW_OK;
}

static void
gst_my_filter_class_init(GstMyFilterClass *klass)
{
    GstElementClass *element_class = GST_ELEMENT_CLASS(klass);
    GstVideoFilterClass *vfilter_class = GST_VIDEO_FILTER_CLASS(klass);

    vfilter_class->transform_frame_ip = gst_my_filter_transform_frame_ip;

    gst_element_class_set_static_metadata(element_class,
        "My Custom Filter",
        "Filter/Effect/Video",
        "My custom video filter",
        "Your Name");

    GstCaps *caps = gst_caps_from_string(GST_VIDEO_CAPS_MAKE("RGB"));
    
    gst_element_class_add_pad_template(element_class,
        gst_pad_template_new("src", GST_PAD_SRC, GST_PAD_ALWAYS, caps));
    gst_element_class_add_pad_template(element_class,
        gst_pad_template_new("sink", GST_PAD_SINK, GST_PAD_ALWAYS, caps));
    
    gst_caps_unref(caps);
}

static void
gst_my_filter_init(GstMyFilter *filter)
{
    filter->threshold = 128;
}

static gboolean
plugin_init(GstPlugin *plugin)
{
    GST_DEBUG_CATEGORY_INIT(gst_my_filter_debug, "myfilter", 0, "My filter");
    return gst_element_register(plugin, "myfilter", GST_RANK_NONE, GST_TYPE_MY_FILTER);
}

GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    myfilter,
    "My custom filter plugin",
    plugin_init,
    "1.0",
    "LGPL",
    "GStreamer",
    "http://gstreamer.net/"
)
```

### Step 3: Create `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.10)
project(gst_my_plugin)

set(CMAKE_CXX_STANDARD 14)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)
find_package(OpenCV REQUIRED)

include_directories(
    ${GSTREAMER_INCLUDE_DIRS}
    ${GST_VIDEO_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(gstmyfilter SHARED gst_my_filter.cpp)

target_link_libraries(gstmyfilter
    ${GSTREAMER_LIBRARIES}
    ${GST_VIDEO_LIBRARIES}
    ${OpenCV_LIBS}
)

install(TARGETS gstmyfilter
    LIBRARY DESTINATION /usr/lib/aarch64-linux-gnu/gstreamer-1.0
)
```

### Step 4: Add to Dockerfile

```dockerfile
RUN cd /ros2_ws/gst_my_plugin && \
    mkdir -p build && cd build && \
    cmake .. && \
    make && \
    make install
```

### Step 5: Use in Pipeline

In your launch file:

```python
gscam_config = 'v4l2src device=/dev/video0 ! videoconvert ! video/x-raw,format=RGB ! myfilter ! videoconvert ! video/x-raw,format=RGB'
```

---

## Common OpenCV Operations

### Edge Detection
```cpp
cv::Mat edges;
cv::Canny(gray, edges, 50, 150);
cv::cvtColor(edges, img, cv::COLOR_GRAY2RGB);
```

### Blur
```cpp
cv::GaussianBlur(img, img, cv::Size(15, 15), 0);
```

### Color Filter
```cpp
cv::Mat hsv;
cv::cvtColor(img, hsv, cv::COLOR_RGB2HSV);
cv::Scalar lower(100, 50, 50);   // Lower bound
cv::Scalar upper(130, 255, 255); // Upper bound
cv::Mat mask;
cv::inRange(hsv, lower, upper, mask);
cv::bitwise_and(img, img, img, mask);
```

### Brightness/Contrast
```cpp
img.convertTo(img, -1, 1.5, 20);  // alpha=1.5 (contrast), beta=20 (brightness)
```

### Draw Shapes
```cpp
cv::rectangle(img, cv::Point(10, 10), cv::Point(100, 100), cv::Scalar(0, 255, 0), 2);
cv::circle(img, cv::Point(50, 50), 20, cv::Scalar(255, 0, 0), -1);
cv::putText(img, "Hello", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
```

---

## Examples of Existing Plugins

### 1. **opencvface** - Face Detection
- **Location:** `gst_mediapipe_plugin/gst_opencv_face.cpp`
- **Element name:** `opencvface`
- **Function:** Detects faces using Haar Cascade
- **Use:** `! opencvface !`

### 2. **opencvoptflow** - Optical Flow
- **Location:** `gst_opencv_optflow/gst_opencv_optflow.cpp`
- **Element name:** `opencvoptflow`
- **Function:** Dense optical flow visualization
- **Use:** `! opencvoptflow !`

---

## Testing Your Plugin

```bash
# 1. Build
cd /ros2_ws/gst_my_plugin
mkdir build && cd build
cmake ..
make
sudo make install

# 2. Verify it's loaded
gst-inspect-1.0 myfilter

# 3. Test with gst-launch
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! video/x-raw,format=RGB ! myfilter ! videoconvert ! autovideosink

# 4. Use in gscam2 pipeline
# Edit main_gscam.launch.py:
gscam_config = '... ! myfilter ! ...'
```

---

## Quick Plugin Examples

### Example 1: Simple Brightness Adjuster

```cpp
static GstFlowReturn
transform_frame_ip(GstVideoFilter *filter, GstVideoFrame *frame)
{
    guint8 *data = (guint8 *)GST_VIDEO_FRAME_PLANE_DATA(frame, 0);
    gint width = GST_VIDEO_FRAME_WIDTH(frame);
    gint height = GST_VIDEO_FRAME_HEIGHT(frame);
    gint stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);
    
    // Increase brightness by 20
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width * 3; x++) {  // RGB = 3 bytes
            int val = data[y * stride + x] + 20;
            data[y * stride + x] = CLAMP(val, 0, 255);
        }
    }
    
    return GST_FLOW_OK;
}
```

### Example 2: Edge Detection

```cpp
static GstFlowReturn
transform_frame_ip(GstVideoFilter *filter, GstVideoFrame *frame)
{
    gint width = GST_VIDEO_FRAME_WIDTH(frame);
    gint height = GST_VIDEO_FRAME_HEIGHT(frame);
    guint8 *data = (guint8 *)GST_VIDEO_FRAME_PLANE_DATA(frame, 0);
    gint stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);
    
    cv::Mat img(height, width, CV_8UC3, data, stride);
    
    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
    
    // Edge detection
    cv::Mat edges;
    cv::Canny(gray, edges, 50, 150);
    
    // Convert back to RGB
    cv::cvtColor(edges, img, cv::COLOR_GRAY2RGB);
    
    return GST_FLOW_OK;
}
```

### Example 3: Circle Detection

```cpp
static GstFlowReturn
transform_frame_ip(GstVideoFilter *filter, GstVideoFrame *frame)
{
    gint width = GST_VIDEO_FRAME_WIDTH(frame);
    gint height = GST_VIDEO_FRAME_HEIGHT(frame);
    guint8 *data = (guint8 *)GST_VIDEO_FRAME_PLANE_DATA(frame, 0);
    gint stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);
    
    cv::Mat img(height, width, CV_8UC3, data, stride);
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
    
    // Detect circles
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, 20,
                     100, 30, 10, 50);  // min/max radius
    
    // Draw circles
    for (size_t i = 0; i < circles.size(); i++) {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle(img, center, radius, cv::Scalar(0, 255, 0), 2);
    }
    
    return GST_FLOW_OK;
}
```

---

## Build Process

### Manual Build (for testing)

```bash
cd /ros2_ws/gst_my_plugin
mkdir -p build && cd build
cmake ..
make
sudo make install
```

### Automatic Build (Dockerfile)

Add to Dockerfile after line 86:

```dockerfile
RUN cd /ros2_ws/gst_my_plugin && \
    mkdir -p build && cd build && \
    cmake .. && \
    make && \
    make install
```

Then rebuild:
```bash
./scripts/build.sh
```

---

## Debugging Tips

### 1. Check if plugin is loaded
```bash
gst-inspect-1.0 myfilter
```

### 2. Enable GStreamer debug output
```bash
GST_DEBUG=3 gst-launch-1.0 v4l2src ! videoconvert ! myfilter ! fakesink
```

### 3. Print from your plugin
```cpp
GST_INFO("My debug message: %d", some_value);
GST_WARNING("Warning message");
GST_ERROR("Error message");
```

### 4. Test pipeline without ROS
```bash
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! video/x-raw,format=RGB ! myfilter ! videoconvert ! autovideosink
```

---

## Performance Tips

1. **Process in-place** - Modify the frame buffer directly (faster)
2. **Reduce resolution** - Smaller frames = faster processing
3. **Minimize memory allocations** - Reuse cv::Mat objects
4. **Use integer operations** - Avoid floating point when possible
5. **Optimize algorithm parameters** - Reduce iterations, window sizes
6. **Profile your code** - Find bottlenecks

---

## Common Issues

### Plugin not found
- Check installation path: `ls /usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstmyfilter.so`
- Run `gst-inspect-1.0 --print-all | grep myfilter`

### Compilation errors
- Ensure all headers are included
- Check OpenCV is installed: `pkg-config --cflags opencv4`
- Verify GStreamer dev packages: `pkg-config --cflags gstreamer-1.0`

### Pipeline errors
- Use `GST_DEBUG=3` to see detailed errors
- Check video format matches (RGB vs BGR)
- Verify caps (width, height, format)

---

## Summary: Create New Plugin Checklist

- [ ] Create directory: `ros2_ws/gst_my_plugin/`
- [ ] Write `gst_my_filter.cpp` with transform function
- [ ] Write `CMakeLists.txt`
- [ ] Add build step to Dockerfile
- [ ] Run `./scripts/build.sh`
- [ ] Test with `gst-inspect-1.0 myfilter`
- [ ] Use in launch file pipeline
- [ ] Run `./scripts/run.sh`

---

## Available OpenCV Algorithms

- **Feature Detection:** SIFT, SURF, ORB, FAST
- **Object Detection:** Haar Cascades, HOG, Template Matching
- **Tracking:** KCF, CSRT, MedianFlow, Optical Flow
- **Segmentation:** GrabCut, Watershed, K-means
- **Filters:** Bilateral, Median, Gaussian, Morphology
- **Transforms:** Affine, Perspective, Rotation
- **Color:** HSV, LAB, YUV conversions
- **Machine Learning:** SVM, K-NN, Decision Trees

All these can be integrated into GStreamer plugins for real-time video processing!


