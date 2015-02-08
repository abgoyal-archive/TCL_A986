

#ifndef PluginWidget_h
#define PluginWidget_h

#include "Widget.h"
#include "GraphicsLayer.h"

namespace WebCore {

class PluginWidget : public Widget {
    
public:
    PluginWidget(PlatformWidget widget = 0) : Widget(widget) { }
#if USE(ACCELERATED_COMPOSITING)
    PlatformLayer* platformLayer() const;
#endif
    
private:
    virtual void invalidateRect(const IntRect&);
    virtual bool isPluginWidget() const { return true; }
};

#if USE(ACCELERATED_COMPOSITING) && !PLATFORM(MAC)
inline PlatformLayer* PluginWidget::platformLayer() const
{
    return 0;
}
#endif

} // namespace WebCore

#endif // PluginWidget_h
