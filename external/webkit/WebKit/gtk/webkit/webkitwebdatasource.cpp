

#include "config.h"
#include "webkitwebdatasource.h"

#include "ArchiveResource.h"
#include "DocumentLoaderGtk.h"
#include "FrameLoaderClientGtk.h"
#include "FrameLoader.h"
#include "KURL.h"
#include "PlatformString.h"
#include "ResourceRequest.h"
#include "runtime/InitializeThreading.h"
#include "SharedBuffer.h"
#include "SubstituteData.h"
#include "webkitwebresource.h"
#include "webkitprivate.h"
#include "wtf/Assertions.h"

#include <glib.h>


using namespace WebCore;
using namespace WebKit;

struct _WebKitWebDataSourcePrivate {
    WebKit::DocumentLoader* loader;

    WebKitNetworkRequest* initialRequest;
    WebKitNetworkRequest* networkRequest;
    WebKitWebResource* mainresource;

    GString* data;

    gchar* textEncoding;
    gchar* unreachableURL;
};

#define WEBKIT_WEB_DATA_SOURCE_GET_PRIVATE(obj)        (G_TYPE_INSTANCE_GET_PRIVATE((obj), WEBKIT_TYPE_WEB_DATA_SOURCE, WebKitWebDataSourcePrivate))

G_DEFINE_TYPE(WebKitWebDataSource, webkit_web_data_source, G_TYPE_OBJECT);

static void webkit_web_data_source_dispose(GObject* object)
{
    WebKitWebDataSource* webDataSource = WEBKIT_WEB_DATA_SOURCE(object);
    WebKitWebDataSourcePrivate* priv = webDataSource->priv;

    ASSERT(priv->loader);
    ASSERT(!priv->loader->isLoading());
    priv->loader->detachDataSource();
    priv->loader->deref();

    if (priv->initialRequest) {
        g_object_unref(priv->initialRequest);
        priv->initialRequest = NULL;
    }

    if (priv->networkRequest) {
        g_object_unref(priv->networkRequest);
        priv->networkRequest = NULL;
    }

    if (priv->mainresource) {
        g_object_unref(priv->mainresource);
        priv->mainresource = NULL;
    }

    G_OBJECT_CLASS(webkit_web_data_source_parent_class)->dispose(object);
}

static void webkit_web_data_source_finalize(GObject* object)
{
    WebKitWebDataSource* dataSource = WEBKIT_WEB_DATA_SOURCE(object);
    WebKitWebDataSourcePrivate* priv = dataSource->priv;

    g_free(priv->unreachableURL);
    g_free(priv->textEncoding);

    if (priv->data) {
        g_string_free(priv->data, TRUE);
        priv->data = NULL;
    }

    G_OBJECT_CLASS(webkit_web_data_source_parent_class)->finalize(object);
}

static void webkit_web_data_source_class_init(WebKitWebDataSourceClass* klass)
{
    GObjectClass* gobject_class = G_OBJECT_CLASS(klass);
    gobject_class->dispose = webkit_web_data_source_dispose;
    gobject_class->finalize = webkit_web_data_source_finalize;

    webkit_init();

    g_type_class_add_private(gobject_class, sizeof(WebKitWebDataSourcePrivate));
}

static void webkit_web_data_source_init(WebKitWebDataSource* webDataSource)
{
    webDataSource->priv = WEBKIT_WEB_DATA_SOURCE_GET_PRIVATE(webDataSource);
}

WebKitWebDataSource* webkit_web_data_source_new_with_loader(PassRefPtr<WebKit::DocumentLoader> loader)
{
    WebKitWebDataSource* webDataSource = WEBKIT_WEB_DATA_SOURCE(g_object_new(WEBKIT_TYPE_WEB_DATA_SOURCE, NULL));
    WebKitWebDataSourcePrivate* priv = webDataSource->priv;
    priv->loader = loader.releaseRef();

    return webDataSource;
}

WebKitWebDataSource* webkit_web_data_source_new()
{
    WebKitNetworkRequest* request = webkit_network_request_new("about:blank");
    WebKitWebDataSource* datasource = webkit_web_data_source_new_with_request(request);
    g_object_unref(request);

    return datasource;
}

WebKitWebDataSource* webkit_web_data_source_new_with_request(WebKitNetworkRequest* request)
{
    ASSERT(request);

    const gchar* uri = webkit_network_request_get_uri(request);

    WebKitWebDataSource* datasource;
    datasource = webkit_web_data_source_new_with_loader(
        WebKit::DocumentLoader::create(ResourceRequest(KURL(KURL(), String::fromUTF8(uri))),
                                       SubstituteData()));

    WebKitWebDataSourcePrivate* priv = datasource->priv;
    priv->initialRequest = request;

    return datasource;
}

WebKitWebFrame* webkit_web_data_source_get_web_frame(WebKitWebDataSource* webDataSource)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_DATA_SOURCE(webDataSource), NULL);

    WebKitWebDataSourcePrivate* priv = webDataSource->priv;
    FrameLoader* frameLoader = priv->loader->frameLoader();

    if (!frameLoader)
        return NULL;

    return static_cast<WebKit::FrameLoaderClient*>(frameLoader->client())->webFrame();
}

WebKitNetworkRequest* webkit_web_data_source_get_initial_request(WebKitWebDataSource* webDataSource)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_DATA_SOURCE(webDataSource), NULL);

    WebKitWebDataSourcePrivate* priv = webDataSource->priv;
    ResourceRequest request = priv->loader->originalRequest();

    if (priv->initialRequest)
        g_object_unref(priv->initialRequest);

    priv->initialRequest = webkit_network_request_new_with_core_request(request);
    return priv->initialRequest;
}

WebKitNetworkRequest* webkit_web_data_source_get_request(WebKitWebDataSource* webDataSource)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_DATA_SOURCE(webDataSource), NULL);

    WebKitWebDataSourcePrivate* priv = webDataSource->priv;
    FrameLoader* frameLoader = priv->loader->frameLoader();
    if (!frameLoader || !frameLoader->frameHasLoaded())
        return NULL;

    ResourceRequest request = priv->loader->request();

     if (priv->networkRequest)
         g_object_unref(priv->networkRequest);

     priv->networkRequest = webkit_network_request_new_with_core_request(request);
     return priv->networkRequest;
}

G_CONST_RETURN gchar* webkit_web_data_source_get_encoding(WebKitWebDataSource* webDataSource)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_DATA_SOURCE(webDataSource), NULL);

    WebKitWebDataSourcePrivate* priv = webDataSource->priv;
    String textEncodingName = priv->loader->overrideEncoding();

    if (!textEncodingName)
        textEncodingName = priv->loader->response().textEncodingName();

    CString encoding = textEncodingName.utf8();
    g_free(priv->textEncoding);
    priv->textEncoding = g_strdup(encoding.data());
    return priv->textEncoding;
}

gboolean webkit_web_data_source_is_loading(WebKitWebDataSource* webDataSource)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_DATA_SOURCE(webDataSource), NULL);

    WebKitWebDataSourcePrivate* priv = webDataSource->priv;

    return priv->loader->isLoadingInAPISense();
}

GString* webkit_web_data_source_get_data(WebKitWebDataSource* webDataSource)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_DATA_SOURCE(webDataSource), NULL);

    WebKitWebDataSourcePrivate* priv = webDataSource->priv;

    RefPtr<SharedBuffer> mainResourceData = priv->loader->mainResourceData();

    if (!mainResourceData)
        return NULL;

    if (priv->data) {
        g_string_free(priv->data, TRUE);
        priv->data = NULL;
    }

    priv->data = g_string_new_len(mainResourceData->data(), mainResourceData->size());
    return priv->data;
}

WebKitWebResource* webkit_web_data_source_get_main_resource(WebKitWebDataSource* webDataSource)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_DATA_SOURCE(webDataSource), NULL);

    WebKitWebDataSourcePrivate* priv = webDataSource->priv;

    if (priv->mainresource)
        return priv->mainresource;

    WebKitWebFrame* webFrame = webkit_web_data_source_get_web_frame(webDataSource);
    WebKitWebView* webView = getViewFromFrame(webFrame);

    priv->mainresource = WEBKIT_WEB_RESOURCE(g_object_ref(webkit_web_view_get_main_resource(webView)));

    return priv->mainresource;
}

G_CONST_RETURN gchar* webkit_web_data_source_get_unreachable_uri(WebKitWebDataSource* webDataSource)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_DATA_SOURCE(webDataSource), NULL);

    WebKitWebDataSourcePrivate* priv = webDataSource->priv;
    const KURL& unreachableURL = priv->loader->unreachableURL();

    if (unreachableURL.isEmpty())
        return NULL;

    g_free(priv->unreachableURL);
    priv->unreachableURL = g_strdup(unreachableURL.string().utf8().data());
    return priv->unreachableURL;
}

GList* webkit_web_data_source_get_subresources(WebKitWebDataSource* webDataSource)
{
    g_return_val_if_fail(WEBKIT_IS_WEB_DATA_SOURCE(webDataSource), NULL);

    WebKitWebFrame* webFrame = webkit_web_data_source_get_web_frame(webDataSource);
    WebKitWebView* webView = getViewFromFrame(webFrame);

    return webkit_web_view_get_subresources(webView);
}
