

#ifndef webkitnewtorkresponse_h
#define webkitnewtorkresponse_h

#include <glib-object.h>
#include <libsoup/soup.h>

#include <webkit/webkitdefines.h>

G_BEGIN_DECLS

#define WEBKIT_TYPE_NETWORK_RESPONSE            (webkit_network_response_get_type())
#define WEBKIT_NETWORK_RESPONSE(obj)            (G_TYPE_CHECK_INSTANCE_CAST((obj), WEBKIT_TYPE_NETWORK_RESPONSE, WebKitNetworkResponse))
#define WEBKIT_NETWORK_RESPONSE_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST((klass),  WEBKIT_TYPE_NETWORK_RESPONSE, WebKitNetworkResponseClass))
#define WEBKIT_IS_NETWORK_RESPONSE(obj)         (G_TYPE_CHECK_INSTANCE_TYPE((obj), WEBKIT_TYPE_NETWORK_RESPONSE))
#define WEBKIT_IS_NETWORK_RESPONSE_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass),  WEBKIT_TYPE_NETWORK_RESPONSE))
#define WEBKIT_NETWORK_RESPONSE_GET_CLASS(obj)  (G_TYPE_INSTANCE_GET_CLASS((obj),  WEBKIT_TYPE_NETWORK_RESPONSE, WebKitNetworkResponseClass))

typedef struct _WebKitNetworkResponsePrivate WebKitNetworkResponsePrivate;

struct _WebKitNetworkResponse {
    GObject parent_instance;

    /*< private >*/
    WebKitNetworkResponsePrivate *priv;
};

struct _WebKitNetworkResponseClass {
    GObjectClass parent_class;

    /* Padding for future expansion */
    void (*_webkit_reserved0) (void);
    void (*_webkit_reserved1) (void);
    void (*_webkit_reserved2) (void);
    void (*_webkit_reserved3) (void);
};

WEBKIT_API GType
webkit_network_response_get_type (void);

WEBKIT_API WebKitNetworkResponse *
webkit_network_response_new      (const gchar          *uri);

WEBKIT_API void
webkit_network_response_set_uri  (WebKitNetworkResponse *response,
                                  const gchar*          uri);

WEBKIT_API G_CONST_RETURN gchar *
webkit_network_response_get_uri  (WebKitNetworkResponse *response);

WEBKIT_API SoupMessage *
webkit_network_response_get_message(WebKitNetworkResponse* response);

G_END_DECLS

#endif
