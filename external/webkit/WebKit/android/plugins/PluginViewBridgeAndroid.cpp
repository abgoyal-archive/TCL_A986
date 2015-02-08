

#include "config.h"
#include "PluginViewBridgeAndroid.h"

namespace WebCore {

    void PluginViewBridgeAndroid::draw(GraphicsContext* gc,
                                       const IntRect& rect) {}

    bool PluginViewBridgeAndroid::forPluginView() const {
        return true;
    }

}
