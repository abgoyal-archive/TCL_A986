

#define LOG_TAG "WebCore"

#include "config.h"
#include "Screen.h"

// This include must come first.
#undef LOG // FIXME: Still have to do this to get the log to show up
#include "utils/Log.h"

#include "FloatRect.h"
#include "Widget.h"
#include <ui/DisplayInfo.h>
#include <ui/PixelFormat.h>
#include <surfaceflinger/SurfaceComposerClient.h>
#include "ScrollView.h"
#include "WebCoreViewBridge.h"

namespace WebCore {

int screenDepth(Widget* page)
{
    android::DisplayInfo info;
    android::SurfaceComposerClient::getDisplayInfo(android::DisplayID(0), &info);
    return info.pixelFormatInfo.bitsPerPixel;
}

int screenDepthPerComponent(Widget* page)
{
    android::DisplayInfo info;
    android::SurfaceComposerClient::getDisplayInfo(android::DisplayID(0), &info);
    return info.pixelFormatInfo.bitsPerPixel;
}

bool screenIsMonochrome(Widget* page)
{
    return false;
}

// The only place I have seen these values used is
// positioning popup windows. If we support multiple windows
// they will be most likely full screen. Therefore,
// the accuracy of these number are not too important.
FloatRect screenRect(Widget* page)
{
    IntRect rect = page->root()->platformWidget()->getBounds();
    return FloatRect(0.0, 0.0, rect.width(), rect.height());
}

// Similar screenRect, this function is commonly used by javascripts
// to position and resize windows (usually to full screen). 
FloatRect screenAvailableRect(Widget* page)
{
    IntRect rect = page->root()->platformWidget()->getBounds();
    return FloatRect(0.0, 0.0, rect.width(), rect.height());
}

} // namespace WebCore
