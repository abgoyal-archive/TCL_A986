

#include "config.h"
#include "Screen.h"

#include "FloatRect.h"
#include "Frame.h"
#include "FrameView.h"
#include "IntRect.h"
#include "NotImplemented.h"
#include "Page.h"
#include "Widget.h"

#include <AEEAppGen.h>
#include <AEEStdLib.h>

namespace WebCore {

struct DisplayInfo {
    int width;
    int height;
    int depth;
};

static void getDisplayInfo(DisplayInfo& info)
{
    IDisplay* display = reinterpret_cast<AEEApplet*>(GETAPPINSTANCE())->m_pIDisplay;
    IBitmap* bitmap = IDisplay_GetDestination(display);
    ASSERT(bitmap);

    AEEBitmapInfo bitmapInfo;
    IBitmap_GetInfo(bitmap, &bitmapInfo, sizeof(AEEBitmapInfo));

    info.width  = bitmapInfo.cx;
    info.height = bitmapInfo.cy;
    info.depth  = bitmapInfo.nDepth;

    IBitmap_Release(bitmap);
}

FloatRect screenRect(Widget*)
{
    DisplayInfo info;
    getDisplayInfo(info);

    return FloatRect(0, 0, info.width, info.height);
}

FloatRect screenAvailableRect(Widget* widget)
{
    return screenRect(widget);
}

int screenDepth(Widget*)
{
    DisplayInfo info;
    getDisplayInfo(info);

    return info.depth;
}

int screenDepthPerComponent(Widget* widget)
{
    return screenDepth(widget);
}

bool screenIsMonochrome(Widget*)
{
    return false;
}

} // namespace WebCore

