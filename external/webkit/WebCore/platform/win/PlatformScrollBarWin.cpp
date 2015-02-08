

#include "config.h"
#include "PlatformScrollBar.h"

#include "FrameView.h"
#include "ScrollbarClient.h"
#include "ScrollbarTheme.h"

namespace WebCore {

PlatformScrollbar::PlatformScrollbar(ScrollbarClient* client, ScrollbarOrientation orientation, ScrollbarControlSize size,
                                     ScrollbarTheme* theme)
    : Scrollbar(client, orientation, size, theme)
{
}

}

