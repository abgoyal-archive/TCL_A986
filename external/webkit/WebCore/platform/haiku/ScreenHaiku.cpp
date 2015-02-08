

#include "config.h"
#include "Screen.h"

#include "FloatRect.h"
#include "Frame.h"
#include "FrameView.h"
#include "IntRect.h"
#include "NotImplemented.h"
#include "Page.h"
#include "Widget.h"
#include <GraphicsDefs.h>
#include <interface/Screen.h>


namespace WebCore {

FloatRect screenRect(Widget*)
{
    BScreen screen;
    // FIXME: We assume this screen is valid
    return FloatRect(screen.Frame());
}

FloatRect screenAvailableRect(Widget* widget)
{
    // FIXME: We could use the get_deskbar_frame() function
    // from InterfaceDefs.h to make this smaller
    return screenRect(widget);
}

int screenDepth(Widget*)
{
    BScreen screen;
    // FIXME: We assume this screen is valid
    color_space cs = screen.ColorSpace();

    size_t pixelChunk, rowAlignment, pixelsPerChunk;
    if (get_pixel_size_for(cs, &pixelChunk, &rowAlignment, &pixelsPerChunk) == B_OK)
        // FIXME: Not sure if this is right
        return pixelChunk * 8;

    return 8;
}

int screenDepthPerComponent(Widget*)
{
    notImplemented();
    return 8;
}

bool screenIsMonochrome(Widget*)
{
    BScreen screen;
    // FIXME: We assume this screen is valid
    return screen.ColorSpace() == B_MONOCHROME_1_BIT;
}

} // namespace WebCore

