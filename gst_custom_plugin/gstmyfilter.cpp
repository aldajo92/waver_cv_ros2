/*
 * Custom GStreamer Video Filter Plugin
 * Processes video frames directly in the pipeline
 * 
 * Build: g++ -shared -fPIC gstmyfilter.cpp -o libgstmyfilter.so `pkg-config --cflags --libs gstreamer-1.0 gstreamer-video-1.0`
 * Install: cp libgstmyfilter.so /usr/lib/aarch64-linux-gnu/gstreamer-1.0/
 * Use: gst-launch-1.0 v4l2src ! myfilter ! ...
 */

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideofilter.h>

GST_DEBUG_CATEGORY_STATIC(gst_my_filter_debug);
#define GST_CAT_DEFAULT gst_my_filter_debug

#define GST_TYPE_MY_FILTER (gst_my_filter_get_type())
#define GST_MY_FILTER(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_MY_FILTER,GstMyFilter))

typedef struct _GstMyFilter GstMyFilter;
typedef struct _GstMyFilterClass GstMyFilterClass;

struct _GstMyFilter {
    GstVideoFilter videofilter;
    // Add your properties here
    gint threshold;
};

struct _GstMyFilterClass {
    GstVideoFilterClass parent_class;
};

GType gst_my_filter_get_type(void);

#define gst_my_filter_parent_class parent_class
G_DEFINE_TYPE(GstMyFilter, gst_my_filter, GST_TYPE_VIDEO_FILTER);

// Process each frame
static GstFlowReturn
gst_my_filter_transform_frame_ip(GstVideoFilter *filter, GstVideoFrame *frame)
{
    GstMyFilter *myfilter = GST_MY_FILTER(filter);
    guint8 *data;
    gint width, height, stride;
    gint i, j;

    width = GST_VIDEO_FRAME_WIDTH(frame);
    height = GST_VIDEO_FRAME_HEIGHT(frame);
    stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);
    data = (guint8 *)GST_VIDEO_FRAME_PLANE_DATA(frame, 0);

    // Example: Simple brightness adjustment
    for (i = 0; i < height; i++) {
        for (j = 0; j < width * 3; j++) {  // RGB = 3 bytes per pixel
            gint val = data[i * stride + j] + 20;  // Add brightness
            data[i * stride + j] = CLAMP(val, 0, 255);
        }
    }

    // Example: Edge detection (simplified)
    // for (i = 1; i < height - 1; i++) {
    //     for (j = 1; j < width - 1; j++) {
    //         gint idx = i * stride + j * 3;
    //         gint dx = abs(data[idx] - data[idx + 3]);
    //         gint dy = abs(data[idx] - data[idx + stride]);
    //         gint edge = dx + dy;
    //         data[idx] = data[idx + 1] = data[idx + 2] = CLAMP(edge, 0, 255);
    //     }
    // }

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
        "Filter/Video",
        "Custom video processing filter",
        "Your Name <your@email.com>");

    GstCaps *caps = gst_caps_from_string(
        GST_VIDEO_CAPS_MAKE("{ RGB, BGR }"));
    
    GstPadTemplate *src_template = gst_pad_template_new(
        "src", GST_PAD_SRC, GST_PAD_ALWAYS, caps);
    GstPadTemplate *sink_template = gst_pad_template_new(
        "sink", GST_PAD_SINK, GST_PAD_ALWAYS, caps);
    
    gst_element_class_add_pad_template(element_class, src_template);
    gst_element_class_add_pad_template(element_class, sink_template);
    
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
    GST_DEBUG_CATEGORY_INIT(gst_my_filter_debug, "myfilter", 0, "My custom filter");
    return gst_element_register(plugin, "myfilter", GST_RANK_NONE, GST_TYPE_MY_FILTER);
}

GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    myfilter,
    "My custom video filter plugin",
    plugin_init,
    "1.0",
    "LGPL",
    "GStreamer",
    "http://gstreamer.net/"
)

