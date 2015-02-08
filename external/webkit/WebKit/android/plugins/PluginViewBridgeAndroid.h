

#ifndef PluginViewBridgeAndroid_H
#define PluginViewBridgeAndroid_H

#include "PluginView.h"
#include "WebCoreViewBridge.h"

namespace WebCore {

    // (Dummy for now) WebCoreViewBridge associated with a PluginView Widget.
    class PluginViewBridgeAndroid : public WebCoreViewBridge {
    public:
        PluginViewBridgeAndroid() {}

        // overrides
        virtual void draw(GraphicsContext* gc, const IntRect& rect);
        virtual bool forPluginView() const;
    };

} // namespace WebCore

#endif 
