

#ifndef webkitwebwindowfeatures_h
#define webkitwebwindowfeatures_h

#include <glib-object.h>

#include <webkit/webkitdefines.h>

G_BEGIN_DECLS

#define WEBKIT_TYPE_WEB_WINDOW_FEATURES            (webkit_web_window_features_get_type())
#define WEBKIT_WEB_WINDOW_FEATURES(obj)            (G_TYPE_CHECK_INSTANCE_CAST((obj), WEBKIT_TYPE_WEB_WINDOW_FEATURES, WebKitWebWindowFeatures))
#define WEBKIT_WEB_WINDOW_FEATURES_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST((klass),  WEBKIT_TYPE_WEB_WINDOW_FEATURES, WebKitWebWindowFeaturesClass))
#define WEBKIT_IS_WEB_WINDOW_FEATURES(obj)         (G_TYPE_CHECK_INSTANCE_TYPE((obj), WEBKIT_TYPE_WEB_WINDOW_FEATURES))
#define WEBKIT_IS_WEB_WINDOW_FEATURES_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass),  WEBKIT_TYPE_WEB_WINDOW_FEATURES))
#define WEBKIT_WEB_WINDOW_FEATURES_GET_CLASS(obj)  (G_TYPE_INSTANCE_GET_CLASS((obj),  WEBKIT_TYPE_WEB_WINDOW_FEATURES, WebKitWebWindowFeaturesClass))

typedef struct _WebKitWebWindowFeaturesPrivate WebKitWebWindowFeaturesPrivate;

struct _WebKitWebWindowFeatures {
    GObject parent_instance;

    /*< private >*/
    WebKitWebWindowFeaturesPrivate* priv;
};

struct _WebKitWebWindowFeaturesClass {
    GObjectClass parent_class;

    /*< private >*/
    /* Padding for future expansion */
    void (*_webkit_reserved1) (void);
    void (*_webkit_reserved2) (void);
    void (*_webkit_reserved3) (void);
    void (*_webkit_reserved4) (void);
};

WEBKIT_API GType
webkit_web_window_features_get_type            (void);

WEBKIT_API WebKitWebWindowFeatures*
webkit_web_window_features_new                 (void);

WEBKIT_API gboolean
webkit_web_window_features_equal               (WebKitWebWindowFeatures* features1,
                                                WebKitWebWindowFeatures* features2);

G_END_DECLS

#endif /* webkitwebwindowfeatures_h */
