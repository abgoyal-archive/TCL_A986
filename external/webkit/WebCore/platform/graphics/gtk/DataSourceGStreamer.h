

#ifndef DATA_SOURCE_GSTREAMER_H
#define DATA_SOURCE_GSTREAMER_H

#include <glib-object.h>
#include <gst/base/gstbasesrc.h>

G_BEGIN_DECLS

#define WEBKIT_TYPE_DATA_SRC            (webkit_data_src_get_type ())
#define WEBKIT_DATA_SRC(obj)            (G_TYPE_CHECK_INSTANCE_CAST ((obj), WEBKIT_TYPE_DATA_SRC, WebkitDataSrc))
#define WEBKIT_DATA_SRC_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST ((klass), WEBKIT_TYPE_DATA_SRC, WebkitDataSrcClass))
#define WEBKIT_IS_DATA_SRC(obj)         (G_TYPE_CHECK_INSTANCE_TYPE ((obj), WEBKIT_TYPE_DATA_SRC))
#define WEBKIT_IS_DATA_SRC_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE ((klass), WEBKIT_TYPE_DATA_SRC))

typedef struct _WebkitDataSrc        WebkitDataSrc;
typedef struct _WebkitDataSrcClass   WebkitDataSrcClass;


struct _WebkitDataSrc {
    GstBin parent;

    /* explicit pointers to stuff used */
    GstElement* kid;
    GstPad* pad;
    gchar* uri;
};

struct _WebkitDataSrcClass {
    GstBinClass parent_class;
};

GType webkit_data_src_get_type(void);

G_END_DECLS

#endif
