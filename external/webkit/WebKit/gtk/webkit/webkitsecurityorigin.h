

#ifndef webkitsecurityorigin_h
#define webkitsecurityorigin_h

#include "webkitwebdatabase.h"

G_BEGIN_DECLS

#define WEBKIT_TYPE_SECURITY_ORIGIN             (webkit_security_origin_get_type())
#define WEBKIT_SECURITY_ORIGIN(obj)             (G_TYPE_CHECK_INSTANCE_CAST((obj), WEBKIT_TYPE_SECURITY_ORIGIN, WebKitSecurityOrigin))
#define WEBKIT_SECURITY_ORIGIN_CLASS(klass)     (G_TYPE_CHECK_CLASS_CAST((klass),  WEBKIT_TYPE_SECURITY_ORIGIN, WebKitSecurityOriginClass))
#define WEBKIT_IS_SECURITY_ORIGIN(obj)          (G_TYPE_CHECK_INSTANCE_TYPE((obj), WEBKIT_TYPE_SECURITY_ORIGIN))
#define WEBKIT_IS_SECURITY_ORIGIN_CLASS(klass)  (G_TYPE_CHECK_CLASS_TYPE((klass),  WEBKIT_TYPE_SECURITY_ORIGIN))
#define WEBKIT_SECURITY_ORIGIN_GET_CLASS(obj)   (G_TYPE_INSTANCE_GET_CLASS((obj),  WEBKIT_TYPE_SECURITY_ORIGIN, WebKitSecurityOriginClass))

typedef struct _WebKitSecurityOriginPrivate WebKitSecurityOriginPrivate;

struct _WebKitSecurityOrigin {
    GObject parent_instance;

    /*< private >*/
    WebKitSecurityOriginPrivate* priv;
};

struct _WebKitSecurityOriginClass {
    GObjectClass parent_class;

    /* Padding for future expansion */
    void (*_webkit_reserved1) (void);
    void (*_webkit_reserved2) (void);
    void (*_webkit_reserved3) (void);
    void (*_webkit_reserved4) (void);
};

WEBKIT_API GType
webkit_security_origin_get_type (void);

WEBKIT_API G_CONST_RETURN gchar*
webkit_security_origin_get_protocol           (WebKitSecurityOrigin* securityOrigin);

WEBKIT_API G_CONST_RETURN gchar*
webkit_security_origin_get_host               (WebKitSecurityOrigin* securityOrigin);

WEBKIT_API guint
webkit_security_origin_get_port               (WebKitSecurityOrigin* securityOrigin);

WEBKIT_API guint64
webkit_security_origin_get_web_database_usage (WebKitSecurityOrigin* securityOrigin);

WEBKIT_API guint64
webkit_security_origin_get_web_database_quota (WebKitSecurityOrigin* securityOrigin);

WEBKIT_API void 
webkit_security_origin_set_web_database_quota (WebKitSecurityOrigin* securityOrigin, guint64 quota);

WEBKIT_API GList *
webkit_security_origin_get_all_web_databases  (WebKitSecurityOrigin* securityOrigin);

G_END_DECLS

#endif /* __WEBKIT_SECURITY_ORIGIN_H__ */
