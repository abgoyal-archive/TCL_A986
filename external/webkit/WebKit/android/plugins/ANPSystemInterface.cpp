

// must include config.h first for webkit to fiddle with new/delete
#include "config.h"

#include "CString.h"
#include "JavaSharedClient.h"
#include "PluginClient.h"
#include "PluginPackage.h"
#include "PluginView.h"
#include "PluginWidgetAndroid.h"
#include "SkString.h"
#include "WebViewCore.h"

#include "ANPSystem_npapi.h"

static const char* gApplicationDataDir = NULL;

using namespace android;

static const char* anp_getApplicationDataDirectory() {
    if (NULL == gApplicationDataDir) {
        PluginClient* client = JavaSharedClient::GetPluginClient();
        if (!client)
            return NULL;

        WebCore::String path = client->getPluginSharedDataDirectory();
        int length = path.length();
        if (length == 0)
            return NULL;

        char* storage = (char*) malloc(length + 1);
        if (NULL == storage)
            return NULL;

        memcpy(storage, path.utf8().data(), length);
        storage[length] = '\0';

        // save this assignment for last, so that if multiple threads call us
        // (which should never happen), we never return an incomplete global.
        // At worst, we would allocate storage for the path twice.
        gApplicationDataDir = storage;
    }
    return gApplicationDataDir;
}

static WebCore::PluginView* pluginViewForInstance(NPP instance) {
    if (instance && instance->ndata)
        return static_cast<WebCore::PluginView*>(instance->ndata);
    return PluginView::currentPluginView();
}

static jclass anp_loadJavaClass(NPP instance, const char* className) {
    WebCore::PluginView* pluginView = pluginViewForInstance(instance);
    PluginWidgetAndroid* pluginWidget = pluginView->platformPluginWidget();

    jclass result;
    result = pluginWidget->webViewCore()->getPluginClass(pluginView->plugin()->path(),
                                                         className);
    return result;
}

///////////////////////////////////////////////////////////////////////////////

#define ASSIGN(obj, name)   (obj)->name = anp_##name

void ANPSystemInterfaceV0_Init(ANPInterface* v) {
    ANPSystemInterfaceV0* i = reinterpret_cast<ANPSystemInterfaceV0*>(v);

    ASSIGN(i, getApplicationDataDirectory);
    ASSIGN(i, loadJavaClass);
}
