

#include "config.h"
#include "PlatformWheelEvent.h"
#include "Scrollbar.h"

#include <gdk/gdk.h>

namespace WebCore {

// Keep this in sync with the other platform event constructors
PlatformWheelEvent::PlatformWheelEvent(GdkEventScroll* event)
{
    static const float delta = 1;

    m_deltaX = 0;
    m_deltaY = 0;

    // Docs say an upwards scroll (away from the user) has a positive delta
    switch (event->direction) {
        case GDK_SCROLL_UP:
            m_deltaY = delta;
            break;
        case GDK_SCROLL_DOWN:
            m_deltaY = -delta;
            break;
        case GDK_SCROLL_LEFT:
            m_deltaX = delta;
            break;
        case GDK_SCROLL_RIGHT:
            m_deltaX = -delta;
            break;
    }
    m_wheelTicksX = m_deltaX;
    m_wheelTicksY = m_deltaY;

    m_position = IntPoint(static_cast<int>(event->x), static_cast<int>(event->y));
    m_globalPosition = IntPoint(static_cast<int>(event->x_root), static_cast<int>(event->y_root));
    m_granularity = ScrollByPixelWheelEvent;
    m_isAccepted = false;
    m_shiftKey = event->state & GDK_SHIFT_MASK;
    m_ctrlKey = event->state & GDK_CONTROL_MASK;
    m_altKey = event->state & GDK_MOD1_MASK;
    m_metaKey = event->state & GDK_META_MASK;

    // FIXME: retrieve the user setting for the number of lines to scroll on each wheel event
    m_deltaX *= static_cast<float>(Scrollbar::pixelsPerLineStep());
    m_deltaY *= static_cast<float>(Scrollbar::pixelsPerLineStep());
}

}
