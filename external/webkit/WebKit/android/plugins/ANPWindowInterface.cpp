

// must include config.h first for webkit to fiddle with new/delete
#include "config.h"
#include "SkANP.h"
#include "WebViewCore.h"
#include "PluginView.h"
#include "PluginWidgetAndroid.h"

static PluginView* pluginViewForInstance(NPP instance) {
    if (instance && instance->ndata)
        return static_cast<PluginView*>(instance->ndata);
    return PluginView::currentPluginView();
}

static void anp_setVisibleRects(NPP instance, const ANPRectI rects[], int32_t count) {
    PluginView* pluginView = pluginViewForInstance(instance);
    PluginWidgetAndroid* pluginWidget = pluginView->platformPluginWidget();
    pluginWidget->setVisibleRects(rects, count);
}

static void anp_clearVisibleRects(NPP instance) {
    anp_setVisibleRects(instance, NULL, 0);
}

static void anp_showKeyboard(NPP instance, bool value) {
    PluginView* pluginView = pluginViewForInstance(instance);
    PluginWidgetAndroid* pluginWidget = pluginView->platformPluginWidget();
    if(pluginWidget->hasFocus())
        pluginWidget->webViewCore()->requestKeyboard(value);
}

static void anp_requestFullScreen(NPP instance) {
    PluginView* pluginView = pluginViewForInstance(instance);
    // call focusPluginElement() so that the pluginView receives keyboard events
    pluginView->focusPluginElement();
    PluginWidgetAndroid* pluginWidget = pluginView->platformPluginWidget();
    pluginWidget->requestFullScreen();
}

static void anp_exitFullScreen(NPP instance) {
    PluginView* pluginView = pluginViewForInstance(instance);
    PluginWidgetAndroid* pluginWidget = pluginView->platformPluginWidget();
    pluginWidget->exitFullScreen(true);
}

static void anp_requestCenterFitZoom(NPP instance) {
    PluginView* pluginView = pluginViewForInstance(instance);
    PluginWidgetAndroid* pluginWidget = pluginView->platformPluginWidget();
    pluginWidget->requestCenterFitZoom();
}

///////////////////////////////////////////////////////////////////////////////

#define ASSIGN(obj, name)   (obj)->name = anp_##name

void ANPWindowInterfaceV0_Init(ANPInterface* value) {
    ANPWindowInterfaceV0* i = reinterpret_cast<ANPWindowInterfaceV0*>(value);

    ASSIGN(i, setVisibleRects);
    ASSIGN(i, clearVisibleRects);
    ASSIGN(i, showKeyboard);
    ASSIGN(i, requestFullScreen);
    ASSIGN(i, exitFullScreen);
    ASSIGN(i, requestCenterFitZoom);
}
