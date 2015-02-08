

#include "config.h"
#include "PlatformScreen.h"

#include "ChromiumBridge.h"
#include "IntRect.h"

namespace WebCore {

int screenDepth(Widget* widget)
{
    return ChromiumBridge::screenDepth(widget);
}

int screenDepthPerComponent(Widget* widget)
{
    return ChromiumBridge::screenDepthPerComponent(widget);
}

bool screenIsMonochrome(Widget* widget)
{
    return ChromiumBridge::screenIsMonochrome(widget);
}

FloatRect screenRect(Widget* widget)
{
    return ChromiumBridge::screenRect(widget);
}

FloatRect screenAvailableRect(Widget* widget)
{
    return ChromiumBridge::screenAvailableRect(widget);
}

} // namespace WebCore
