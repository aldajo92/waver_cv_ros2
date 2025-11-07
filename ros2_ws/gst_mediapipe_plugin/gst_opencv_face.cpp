/*
 * GStreamer OpenCV Face Detection Plugin
 * Simpler alternative to MediaPipe using OpenCV Haar Cascades
 * 
 * This is easier to build and works directly in GStreamer pipeline
 */

#define PACKAGE "opencvface"

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideofilter.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>

GST_DEBUG_CATEGORY_STATIC(gst_opencv_face_debug);
#define GST_CAT_DEFAULT gst_opencv_face_debug

#define GST_TYPE_OPENCV_FACE (gst_opencv_face_get_type())
#define GST_OPENCV_FACE(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_OPENCV_FACE,GstOpenCVFace))

typedef struct _GstOpenCVFace GstOpenCVFace;
typedef struct _GstOpenCVFaceClass GstOpenCVFaceClass;

struct _GstOpenCVFace {
    GstVideoFilter videofilter;
    cv::CascadeClassifier face_cascade;
    gboolean initialized;
};

struct _GstOpenCVFaceClass {
    GstVideoFilterClass parent_class;
};

GType gst_opencv_face_get_type(void);

#define gst_opencv_face_parent_class parent_class
G_DEFINE_TYPE(GstOpenCVFace, gst_opencv_face, GST_TYPE_VIDEO_FILTER);

static GstFlowReturn
gst_opencv_face_transform_frame_ip(GstVideoFilter *filter, GstVideoFrame *frame)
{
    GstOpenCVFace *opencv_face = GST_OPENCV_FACE(filter);
    
    // Initialize cascade on first frame
    if (!opencv_face->initialized) {
        std::string cascade_path = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml";
        if (!opencv_face->face_cascade.load(cascade_path)) {
            GST_ERROR("Failed to load face cascade from: %s", cascade_path.c_str());
            return GST_FLOW_ERROR;
        }
        opencv_face->initialized = TRUE;
        GST_INFO("Face cascade loaded successfully");
    }
    
    gint width = GST_VIDEO_FRAME_WIDTH(frame);
    gint height = GST_VIDEO_FRAME_HEIGHT(frame);
    guint8 *data = (guint8 *)GST_VIDEO_FRAME_PLANE_DATA(frame, 0);
    gint stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);
    
    // Create OpenCV Mat from frame data (RGB format)
    cv::Mat img(height, width, CV_8UC3, data, stride);
    
    // Convert to grayscale for detection
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
    
    // Detect faces
    std::vector<cv::Rect> faces;
    opencv_face->face_cascade.detectMultiScale(
        gray, faces, 1.1, 4, 0, cv::Size(30, 30)
    );
    
    // Draw rectangles around faces
    for (size_t i = 0; i < faces.size(); i++) {
        cv::rectangle(
            img,
            cv::Point(faces[i].x, faces[i].y),
            cv::Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height),
            cv::Scalar(0, 255, 0),  // Green in RGB
            2
        );
        
        // Add text
        std::string text = "Face " + std::to_string(i + 1);
        cv::putText(
            img,
            text,
            cv::Point(faces[i].x, faces[i].y - 10),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 255, 0),
            2
        );
    }
    
    return GST_FLOW_OK;
}

static void
gst_opencv_face_class_init(GstOpenCVFaceClass *klass)
{
    GstElementClass *element_class = GST_ELEMENT_CLASS(klass);
    GstVideoFilterClass *vfilter_class = GST_VIDEO_FILTER_CLASS(klass);

    vfilter_class->transform_frame_ip = gst_opencv_face_transform_frame_ip;

    gst_element_class_set_static_metadata(element_class,
        "OpenCV Face Detection",
        "Filter/Effect/Video",
        "Detect faces using OpenCV Haar Cascades",
        "Waver Robotics");

    GstCaps *caps = gst_caps_from_string(GST_VIDEO_CAPS_MAKE("RGB"));
    
    gst_element_class_add_pad_template(element_class,
        gst_pad_template_new("src", GST_PAD_SRC, GST_PAD_ALWAYS, caps));
    gst_element_class_add_pad_template(element_class,
        gst_pad_template_new("sink", GST_PAD_SINK, GST_PAD_ALWAYS, caps));
    
    gst_caps_unref(caps);
}

static void
gst_opencv_face_init(GstOpenCVFace *filter)
{
    filter->initialized = FALSE;
}

static gboolean
plugin_init(GstPlugin *plugin)
{
    GST_DEBUG_CATEGORY_INIT(gst_opencv_face_debug, "opencvface", 0, "OpenCV face detection");
    return gst_element_register(plugin, "opencvface", GST_RANK_NONE, GST_TYPE_OPENCV_FACE);
}

GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    opencvface,
    "OpenCV face detection filter",
    plugin_init,
    "1.0",
    "LGPL",
    "GStreamer",
    "http://gstreamer.net/"
)

