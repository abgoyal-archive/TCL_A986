

#ifndef webkitwebdatabase_h
#define webkitwebdatabase_h

#include <glib-object.h>

#include <webkit/webkitdefines.h>

G_BEGIN_DECLS

#define WEBKIT_TYPE_WEB_DATABASE             (webkit_web_database_get_type())
#define WEBKIT_WEB_DATABASE(obj)             (G_TYPE_CHECK_INSTANCE_CAST((obj), WEBKIT_TYPE_WEB_DATABASE, WebKitWebDatabase))
#define WEBKIT_WEB_DATABASE_CLASS(klass)     (G_TYPE_CHECK_CLASS_CAST((klass),  WEBKIT_TYPE_WEB_DATABASE, WebKitWebDatabaseClass))
#define WEBKIT_IS_WEB_DATABASE(obj)          (G_TYPE_CHECK_INSTANCE_TYPE((obj), WEBKIT_TYPE_WEB_DATABASE))
#define WEBKIT_IS_WEB_DATABASE_CLASS(klass)  (G_TYPE_CHECK_CLASS_TYPE((klass),  WEBKIT_TYPE_WEB_DATABASE))
#define WEBKIT_WEB_DATABASE_GET_CLASS(obj)   (G_TYPE_INSTANCE_GET_CLASS((obj),  WEBKIT_TYPE_WEB_DATABASE, WebKitWebDatabaseClass))

typedef struct _WebKitWebDatabasePrivate WebKitWebDatabasePrivate;

struct _WebKitWebDatabase {
    GObject parent_instance;

    /*< private >*/
    WebKitWebDatabasePrivate* priv;
};

struct _WebKitWebDatabaseClass {
    GObjectClass parent_class;

    /* Padding for future expansion */
    void (*_webkit_reserved1) (void);
    void (*_webkit_reserved2) (void);
    void (*_webkit_reserved3) (void);
    void (*_webkit_reserved4) (void);
};

WEBKIT_API GType
webkit_web_database_get_type            (void);

WEBKIT_API WebKitSecurityOrigin *
webkit_web_database_get_security_origin (WebKitWebDatabase* webDatabase);

WEBKIT_API G_CONST_RETURN gchar*
webkit_web_database_get_name            (WebKitWebDatabase* webDatabase);

WEBKIT_API G_CONST_RETURN gchar*
webkit_web_database_get_display_name    (WebKitWebDatabase* webDatabase);

WEBKIT_API guint64
webkit_web_database_get_expected_size   (WebKitWebDatabase* webDatabase);

WEBKIT_API guint64
webkit_web_database_get_size            (WebKitWebDatabase* webDatabase);

WEBKIT_API G_CONST_RETURN gchar*
webkit_web_database_get_filename        (WebKitWebDatabase* webDatabase);

WEBKIT_API void
webkit_web_database_remove              (WebKitWebDatabase* webDatabase);

WEBKIT_API void
webkit_remove_all_web_databases         (void);

WEBKIT_API G_CONST_RETURN gchar*
webkit_get_web_database_directory_path  (void);

WEBKIT_API void
webkit_set_web_database_directory_path  (const gchar* path);

WEBKIT_API guint64
webkit_get_default_web_database_quota   (void);

WEBKIT_API void
webkit_set_default_web_database_quota   (guint64 defaultQuota);

G_END_DECLS

#endif /* webkitwebdatabase_h */
