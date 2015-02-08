

#ifndef WebKitWebSourceGStreamer_h
#define WebKitWebSourceGStreamer_h

#include "Frame.h"
#include <gst/gst.h>

G_BEGIN_DECLS

#define WEBKIT_TYPE_WEB_SRC            (webkit_web_src_get_type ())
#define WEBKIT_WEB_SRC(obj)            (G_TYPE_CHECK_INSTANCE_CAST ((obj), WEBKIT_TYPE_WEB_SRC, WebKitWebSrc))
#define WEBKIT_WEB_SRC_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST ((klass), WEBKIT_TYPE_WEB_SRC, WebKitWebSrcClass))
#define WEBKIT_IS_WEB_SRC(obj)         (G_TYPE_CHECK_INSTANCE_TYPE ((obj), WEBKIT_TYPE_WEB_SRC))
#define WEBKIT_IS_WEB_SRC_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE ((klass), WEBKIT_TYPE_WEB_SRC))

typedef struct _WebKitWebSrc        WebKitWebSrc;
typedef struct _WebKitWebSrcClass   WebKitWebSrcClass;
typedef struct _WebKitWebSrcPrivate WebKitWebSrcPrivate;

struct _WebKitWebSrc {
    GstBin parent;

    WebKitWebSrcPrivate *priv;
};

struct _WebKitWebSrcClass {
    GstBinClass parentClass;
};

GType webkit_web_src_get_type(void);
void  webKitWebSrcSetFrame(WebKitWebSrc* src, WebCore::Frame* frame);

G_END_DECLS

#endif
