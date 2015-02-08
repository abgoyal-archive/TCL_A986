

// must include config.h first for webkit to fiddle with new/delete
#include "config.h"
#include "SkANP.h"
#include "WebViewCore.h"
#include "PluginView.h"
#include "PluginWidgetAndroid.h"

#include "JavaSharedClient.h"

using namespace android;

struct WrappedANPEvent {
    WebViewCore*            fWVC;
    PluginWidgetAndroid*    fPWA;
    ANPEvent                fEvent;
};

static void send_anpevent(void* data) {
    WrappedANPEvent* wrapper = static_cast<WrappedANPEvent*>(data);
    WebViewCore* core = wrapper->fWVC;
    PluginWidgetAndroid* widget = wrapper->fPWA;

    // be sure we're still alive before delivering the event
    if (WebViewCore::isInstance(core) && core->isPlugin(widget)) {
        widget->sendEvent(wrapper->fEvent);
    }
    delete wrapper;
}

static void anp_postEvent(NPP instance, const ANPEvent* event) {
    if (instance && instance->ndata && event) {
        PluginView* pluginView = static_cast<PluginView*>(instance->ndata);
        PluginWidgetAndroid* pluginWidget = pluginView->platformPluginWidget();
        WebViewCore* wvc = pluginWidget->webViewCore();

        WrappedANPEvent* wrapper = new WrappedANPEvent;
        // recored these, and recheck that they are valid before delivery
        // in send_anpevent
        wrapper->fWVC = pluginWidget->webViewCore();
        wrapper->fPWA = pluginWidget;
        // make a copy of the event
        wrapper->fEvent = *event;
        JavaSharedClient::EnqueueFunctionPtr(send_anpevent, wrapper);
    }
}

///////////////////////////////////////////////////////////////////////////////

#define ASSIGN(obj, name)   (obj)->name = anp_##name

void ANPEventInterfaceV0_Init(ANPInterface* value) {
    ANPEventInterfaceV0* i = reinterpret_cast<ANPEventInterfaceV0*>(value);

    ASSIGN(i, postEvent);
}
