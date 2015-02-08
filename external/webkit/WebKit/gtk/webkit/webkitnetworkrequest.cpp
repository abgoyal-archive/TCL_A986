

#include "config.h"
#include "webkitnetworkrequest.h"

#include "CString.h"
#include "GOwnPtr.h"
#include "ResourceRequest.h"
#include "webkitprivate.h"

#include <glib/gi18n-lib.h>


G_DEFINE_TYPE(WebKitNetworkRequest, webkit_network_request, G_TYPE_OBJECT);

struct _WebKitNetworkRequestPrivate {
    gchar* uri;
    SoupMessage* message;
};

#define WEBKIT_NETWORK_REQUEST_GET_PRIVATE(obj) (G_TYPE_INSTANCE_GET_PRIVATE((obj), WEBKIT_TYPE_NETWORK_REQUEST, WebKitNetworkRequestPrivate))

enum {
    PROP_0,

    PROP_URI,
    PROP_MESSAGE,
};

static void webkit_network_request_dispose(GObject* object)
{
    WebKitNetworkRequest* request = WEBKIT_NETWORK_REQUEST(object);
    WebKitNetworkRequestPrivate* priv = request->priv;

    if (priv->message) {
        g_object_unref(priv->message);
        priv->message = NULL;
    }

    G_OBJECT_CLASS(webkit_network_request_parent_class)->dispose(object);
}

static void webkit_network_request_finalize(GObject* object)
{
    WebKitNetworkRequest* request = WEBKIT_NETWORK_REQUEST(object);
    WebKitNetworkRequestPrivate* priv = request->priv;

    g_free(priv->uri);

    G_OBJECT_CLASS(webkit_network_request_parent_class)->finalize(object);
}

static void webkit_network_request_get_property(GObject* object, guint propertyID, GValue* value, GParamSpec* pspec)
{
    WebKitNetworkRequest* request = WEBKIT_NETWORK_REQUEST(object);

    switch(propertyID) {
    case PROP_URI:
        g_value_set_string(value, webkit_network_request_get_uri(request));
        break;
    case PROP_MESSAGE:
        g_value_set_object(value, webkit_network_request_get_message(request));
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, propertyID, pspec);
    }
}

static void webkit_network_request_set_property(GObject* object, guint propertyID, const GValue* value, GParamSpec* pspec)
{
    WebKitNetworkRequest* request = WEBKIT_NETWORK_REQUEST(object);
    WebKitNetworkRequestPrivate* priv = request->priv;

    switch(propertyID) {
    case PROP_URI:
        webkit_network_request_set_uri(request, g_value_get_string(value));
        break;
    case PROP_MESSAGE:
        priv->message = SOUP_MESSAGE(g_value_dup_object(value));
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, propertyID, pspec);
    }
}

static void webkit_network_request_class_init(WebKitNetworkRequestClass* requestClass)
{
    GObjectClass* objectClass = G_OBJECT_CLASS(requestClass);

    objectClass->dispose = webkit_network_request_dispose;
    objectClass->finalize = webkit_network_request_finalize;
    objectClass->get_property = webkit_network_request_get_property;
    objectClass->set_property = webkit_network_request_set_property;

    webkit_init();

    /**
     * WebKitNetworkRequest:uri:
     *
     * The URI to which the request will be made.
     *
     * Since: 1.1.10
     */
    g_object_class_install_property(objectClass, PROP_URI,
                                    g_param_spec_string("uri",
                                                        _("URI"),
                                                        _("The URI to which the request will be made."),
                                                        NULL,
                                                        (GParamFlags)(WEBKIT_PARAM_READWRITE)));

    /**
     * WebKitNetworkRequest:message:
     *
     * The #SoupMessage that backs the request.
     *
     * Since: 1.1.10
     */
    g_object_class_install_property(objectClass, PROP_MESSAGE,
                                    g_param_spec_object("message",
                                                        _("Message"),
                                                        _("The SoupMessage that backs the request."),
                                                        SOUP_TYPE_MESSAGE,
                                                        (GParamFlags)(WEBKIT_PARAM_READWRITE|G_PARAM_CONSTRUCT_ONLY)));

    g_type_class_add_private(requestClass, sizeof(WebKitNetworkRequestPrivate));
}

static void webkit_network_request_init(WebKitNetworkRequest* request)
{
    WebKitNetworkRequestPrivate* priv = WEBKIT_NETWORK_REQUEST_GET_PRIVATE(request);
    request->priv = priv;
}

// for internal use only
WebKitNetworkRequest* webkit_network_request_new_with_core_request(const WebCore::ResourceRequest& resourceRequest)
{
    GOwnPtr<SoupMessage> soupMessage(resourceRequest.toSoupMessage());
    if (soupMessage)
        return WEBKIT_NETWORK_REQUEST(g_object_new(WEBKIT_TYPE_NETWORK_REQUEST, "message", soupMessage.get(), NULL));

    return WEBKIT_NETWORK_REQUEST(g_object_new(WEBKIT_TYPE_NETWORK_REQUEST, "uri", resourceRequest.url().string().utf8().data(), NULL));
}

WebKitNetworkRequest* webkit_network_request_new(const gchar* uri)
{
    g_return_val_if_fail(uri, NULL);

    return WEBKIT_NETWORK_REQUEST(g_object_new(WEBKIT_TYPE_NETWORK_REQUEST, "uri", uri, NULL));
}

void webkit_network_request_set_uri(WebKitNetworkRequest* request, const gchar* uri)
{
    g_return_if_fail(WEBKIT_IS_NETWORK_REQUEST(request));
    g_return_if_fail(uri);

    WebKitNetworkRequestPrivate* priv = request->priv;

    if (priv->uri)
        g_free(priv->uri);
    priv->uri = g_strdup(uri);

    if (!priv->message)
        return;

    SoupURI* soupURI = soup_uri_new(uri);
    g_return_if_fail(soupURI);

    soup_message_set_uri(priv->message, soupURI);
    soup_uri_free(soupURI);
}

G_CONST_RETURN gchar* webkit_network_request_get_uri(WebKitNetworkRequest* request)
{
    g_return_val_if_fail(WEBKIT_IS_NETWORK_REQUEST(request), NULL);

    WebKitNetworkRequestPrivate* priv = request->priv;

    if (priv->uri)
        return priv->uri;

    SoupURI* soupURI = soup_message_get_uri(priv->message);
    priv->uri = soup_uri_to_string(soupURI, FALSE);
    return priv->uri;
}

SoupMessage* webkit_network_request_get_message(WebKitNetworkRequest* request)
{
    g_return_val_if_fail(WEBKIT_IS_NETWORK_REQUEST(request), NULL);

    WebKitNetworkRequestPrivate* priv = request->priv;

    return priv->message;
}
