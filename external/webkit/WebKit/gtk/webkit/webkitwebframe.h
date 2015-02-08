

#ifndef webkitwebframe_h
#define webkitwebframe_h

#include <glib-object.h>
#include <gtk/gtk.h>

#include <JavaScriptCore/JSBase.h>

#include <webkit/webkitdefines.h>
#include <webkit/webkitnetworkrequest.h>
#include <webkit/webkitwebdatasource.h>

G_BEGIN_DECLS

#define WEBKIT_TYPE_WEB_FRAME            (webkit_web_frame_get_type())
#define WEBKIT_WEB_FRAME(obj)            (G_TYPE_CHECK_INSTANCE_CAST((obj), WEBKIT_TYPE_WEB_FRAME, WebKitWebFrame))
#define WEBKIT_WEB_FRAME_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST((klass),  WEBKIT_TYPE_WEB_FRAME, WebKitWebFrameClass))
#define WEBKIT_IS_WEB_FRAME(obj)         (G_TYPE_CHECK_INSTANCE_TYPE((obj), WEBKIT_TYPE_WEB_FRAME))
#define WEBKIT_IS_WEB_FRAME_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass),  WEBKIT_TYPE_WEB_FRAME))
#define WEBKIT_WEB_FRAME_GET_CLASS(obj)  (G_TYPE_INSTANCE_GET_CLASS((obj),  WEBKIT_TYPE_WEB_FRAME, WebKitWebFrameClass))

typedef struct _WebKitWebFramePrivate WebKitWebFramePrivate;

struct _WebKitWebFrame {
    GObject parent_instance;

    /*< private >*/
    WebKitWebFramePrivate *priv;
};

struct _WebKitWebFrameClass {
    GObjectClass parent_class;

    /*< public >*/
    void (*_webkit_reserved1) (void);
    void (*_webkit_reserved2) (void);
    void (*_webkit_reserved3) (void);
    void (*_webkit_reserved4) (void);
    void (*_webkit_reserved5) (void);
    void (*_webkit_reserved6) (void);
};

typedef enum {
    WEBKIT_LOAD_PROVISIONAL,
    WEBKIT_LOAD_COMMITTED,
    WEBKIT_LOAD_FINISHED,
    WEBKIT_LOAD_FIRST_VISUALLY_NON_EMPTY_LAYOUT,
    WEBKIT_LOAD_FAILED
} WebKitLoadStatus;

WEBKIT_API GType
webkit_web_frame_get_type           (void);

#ifndef WEBKIT_DISABLE_DEPRECATED
WEBKIT_API WebKitWebFrame *
webkit_web_frame_new                (WebKitWebView        *web_view);
#endif

WEBKIT_API WebKitWebView *
webkit_web_frame_get_web_view       (WebKitWebFrame       *frame);

WEBKIT_API G_CONST_RETURN gchar *
webkit_web_frame_get_name           (WebKitWebFrame       *frame);

WEBKIT_API G_CONST_RETURN gchar *
webkit_web_frame_get_title          (WebKitWebFrame       *frame);

WEBKIT_API G_CONST_RETURN gchar *
webkit_web_frame_get_uri            (WebKitWebFrame       *frame);

WEBKIT_API WebKitWebFrame*
webkit_web_frame_get_parent         (WebKitWebFrame       *frame);

WEBKIT_API void
webkit_web_frame_load_uri           (WebKitWebFrame       *frame,
                                     const gchar          *uri);

WEBKIT_API void
webkit_web_frame_load_string        (WebKitWebFrame       *frame,
                                     const gchar          *content,
                                     const gchar          *mime_type,
                                     const gchar          *encoding,
                                     const gchar          *base_uri);

WEBKIT_API void
webkit_web_frame_load_alternate_string (WebKitWebFrame    *frame,
                                        const gchar       *content,
                                        const gchar       *base_url,
                                        const gchar       *unreachable_url);

WEBKIT_API void
webkit_web_frame_load_request       (WebKitWebFrame       *frame,
                                     WebKitNetworkRequest *request);

WEBKIT_API void
webkit_web_frame_stop_loading       (WebKitWebFrame       *frame);

WEBKIT_API void
webkit_web_frame_reload             (WebKitWebFrame       *frame);

WEBKIT_API WebKitWebFrame *
webkit_web_frame_find_frame         (WebKitWebFrame       *frame,
                                     const gchar          *name);

WEBKIT_API JSGlobalContextRef
webkit_web_frame_get_global_context (WebKitWebFrame       *frame);

WEBKIT_API GtkPrintOperationResult
webkit_web_frame_print_full         (WebKitWebFrame       *frame,
                                     GtkPrintOperation    *operation,
                                     GtkPrintOperationAction action,
                                     GError              **error);

WEBKIT_API void
webkit_web_frame_print              (WebKitWebFrame       *frame);

WEBKIT_API WebKitLoadStatus
webkit_web_frame_get_load_status    (WebKitWebFrame       *frame);

WEBKIT_API GtkPolicyType
webkit_web_frame_get_horizontal_scrollbar_policy (WebKitWebFrame        *frame);

WEBKIT_API GtkPolicyType
webkit_web_frame_get_vertical_scrollbar_policy   (WebKitWebFrame        *frame);

WEBKIT_API WebKitWebDataSource *
webkit_web_frame_get_data_source             (WebKitWebFrame       *frame);

WEBKIT_API WebKitWebDataSource *
webkit_web_frame_get_provisional_data_source (WebKitWebFrame       *frame);

WEBKIT_API WebKitSecurityOrigin*
webkit_web_frame_get_security_origin         (WebKitWebFrame       *frame);

WEBKIT_API WebKitNetworkResponse*
webkit_web_frame_get_network_response        (WebKitWebFrame       *frame);

G_END_DECLS

#endif
