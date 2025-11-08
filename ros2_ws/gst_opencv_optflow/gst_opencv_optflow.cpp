/*
 * GStreamer OpenCV Optical Flow Plugin
 * Dense optical flow using Farneback algorithm in C++
 * 
 * Much faster than Python implementation
 */

#define PACKAGE "opencvoptflow"

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideofilter.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

GST_DEBUG_CATEGORY_STATIC(gst_opencv_optflow_debug);
#define GST_CAT_DEFAULT gst_opencv_optflow_debug

#define GST_TYPE_OPENCV_OPTFLOW (gst_opencv_optflow_get_type())
#define GST_OPENCV_OPTFLOW(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_OPENCV_OPTFLOW,GstOpenCVOptFlow))

typedef struct _GstOpenCVOptFlow GstOpenCVOptFlow;
typedef struct _GstOpenCVOptFlowClass GstOpenCVOptFlowClass;

struct _GstOpenCVOptFlow {
    GstVideoFilter videofilter;
    cv::Mat prev_gray;
    cv::Mat hsv;
    gboolean initialized;
};

struct _GstOpenCVOptFlowClass {
    GstVideoFilterClass parent_class;
};

GType gst_opencv_optflow_get_type(void);

#define gst_opencv_optflow_parent_class parent_class
G_DEFINE_TYPE(GstOpenCVOptFlow, gst_opencv_optflow, GST_TYPE_VIDEO_FILTER);

static GstFlowReturn
gst_opencv_optflow_transform_frame_ip(GstVideoFilter *filter, GstVideoFrame *frame)
{
    GstOpenCVOptFlow *optflow = GST_OPENCV_OPTFLOW(filter);
    
    gint width = GST_VIDEO_FRAME_WIDTH(frame);
    gint height = GST_VIDEO_FRAME_HEIGHT(frame);
    guint8 *data = (guint8 *)GST_VIDEO_FRAME_PLANE_DATA(frame, 0);
    gint stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);
    
    // Create OpenCV Mat from frame data (RGB format)
    cv::Mat img(height, width, CV_8UC3, data, stride);
    
    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
    
    // Initialize HSV image on first frame
    if (!optflow->initialized) {
        optflow->hsv = cv::Mat::zeros(height, width, CV_8UC3);
        optflow->hsv.setTo(cv::Scalar(0, 255, 0));  // Set saturation to max
        optflow->initialized = TRUE;
        GST_INFO("Optical flow initialized");
    }
    
    // Calculate optical flow if we have previous frame
    if (!optflow->prev_gray.empty()) {
        cv::Mat flow;
        
        // Dense optical flow (Farneback algorithm) - Aggressive speed optimization
        cv::calcOpticalFlowFarneback(
            optflow->prev_gray, gray,
            flow,
            0.5,   // pyr_scale
            1,     // levels (minimum - fastest)
            7,     // winsize (smaller - faster)
            1,     // iterations (minimum - fastest)
            5,     // poly_n
            1.0,   // poly_sigma
            0      // flags
        );
        
        // Convert flow to polar coordinates
        cv::Mat flow_parts[2];
        cv::split(flow, flow_parts);
        cv::Mat magnitude, angle;
        cv::cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
        
        // Normalize magnitude
        cv::normalize(magnitude, magnitude, 0, 255, cv::NORM_MINMAX);
        
        // Convert to 8-bit
        magnitude.convertTo(magnitude, CV_8UC1);
        angle.convertTo(angle, CV_8UC1, 0.5);  // Scale angle to 0-180
        
        // Update HSV channels
        std::vector<cv::Mat> hsv_channels(3);
        hsv_channels[0] = angle;      // Hue = direction
        hsv_channels[1] = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));  // Saturation = max
        hsv_channels[2] = magnitude;  // Value = speed
        
        cv::merge(hsv_channels, optflow->hsv);
        
        // Convert HSV to RGB
        cv::Mat flow_rgb;
        cv::cvtColor(optflow->hsv, flow_rgb, cv::COLOR_HSV2RGB);
        
        // Copy result back to frame buffer
        for (int y = 0; y < height; y++) {
            memcpy(data + y * stride, flow_rgb.ptr(y), width * 3);
        }
    }
    
    // Update previous frame
    optflow->prev_gray = gray.clone();
    
    return GST_FLOW_OK;
}

static void
gst_opencv_optflow_class_init(GstOpenCVOptFlowClass *klass)
{
    GstElementClass *element_class = GST_ELEMENT_CLASS(klass);
    GstVideoFilterClass *vfilter_class = GST_VIDEO_FILTER_CLASS(klass);

    vfilter_class->transform_frame_ip = gst_opencv_optflow_transform_frame_ip;

    gst_element_class_set_static_metadata(element_class,
        "OpenCV Optical Flow",
        "Filter/Effect/Video",
        "Dense optical flow using OpenCV Farneback algorithm",
        "Waver Robotics");

    GstCaps *caps = gst_caps_from_string(GST_VIDEO_CAPS_MAKE("RGB"));
    
    gst_element_class_add_pad_template(element_class,
        gst_pad_template_new("src", GST_PAD_SRC, GST_PAD_ALWAYS, caps));
    gst_element_class_add_pad_template(element_class,
        gst_pad_template_new("sink", GST_PAD_SINK, GST_PAD_ALWAYS, caps));
    
    gst_caps_unref(caps);
}

static void
gst_opencv_optflow_init(GstOpenCVOptFlow *filter)
{
    filter->initialized = FALSE;
}

static gboolean
plugin_init(GstPlugin *plugin)
{
    GST_DEBUG_CATEGORY_INIT(gst_opencv_optflow_debug, "opencvoptflow", 0, "OpenCV optical flow");
    return gst_element_register(plugin, "opencvoptflow", GST_RANK_NONE, GST_TYPE_OPENCV_OPTFLOW);
}

GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    opencvoptflow,
    "OpenCV optical flow filter",
    plugin_init,
    "1.0",
    "LGPL",
    "GStreamer",
    "http://gstreamer.net/"
)

