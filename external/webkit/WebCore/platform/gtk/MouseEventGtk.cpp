

#include "config.h"
#include "PlatformMouseEvent.h"

#include "Assertions.h"

#include <gdk/gdk.h>

namespace WebCore {

// FIXME: Would be even better to figure out which modifier is Alt instead of always using GDK_MOD1_MASK.

// Keep this in sync with the other platform event constructors
PlatformMouseEvent::PlatformMouseEvent(GdkEventButton* event)
{
    m_timestamp = event->time;
    m_position = IntPoint((int)event->x, (int)event->y);
    m_globalPosition = IntPoint((int)event->x_root, (int)event->y_root);
    m_shiftKey = event->state & GDK_SHIFT_MASK;
    m_ctrlKey = event->state & GDK_CONTROL_MASK;
    m_altKey = event->state & GDK_MOD1_MASK;
    m_metaKey = event->state & GDK_META_MASK;

    switch (event->type) {
    case GDK_BUTTON_PRESS:
    case GDK_2BUTTON_PRESS:
    case GDK_3BUTTON_PRESS:
    case GDK_BUTTON_RELEASE:
        m_eventType = MouseEventPressed;
        if (event->type == GDK_BUTTON_RELEASE) {
            m_eventType = MouseEventReleased;
            m_clickCount = 0;
        } else if (event->type == GDK_BUTTON_PRESS)
            m_clickCount = 1;
        else if (event->type == GDK_2BUTTON_PRESS)
            m_clickCount = 2;
        else if (event->type == GDK_3BUTTON_PRESS)
            m_clickCount = 3;

        if (event->button == 1)
            m_button = LeftButton;
        else if (event->button == 2)
            m_button = MiddleButton;
        else if (event->button == 3)
            m_button = RightButton;
        break;

    default:
        ASSERT_NOT_REACHED();
    };
}

PlatformMouseEvent::PlatformMouseEvent(GdkEventMotion* motion)
{
    m_timestamp = motion->time;
    m_position = IntPoint((int)motion->x, (int)motion->y);
    m_globalPosition = IntPoint((int)motion->x_root, (int)motion->y_root);
    m_shiftKey = motion->state & GDK_SHIFT_MASK;
    m_ctrlKey = motion->state & GDK_CONTROL_MASK;
    m_altKey = motion->state & GDK_MOD1_MASK;
    m_metaKey = motion->state & GDK_MOD2_MASK;

    switch (motion->type) {
    case GDK_MOTION_NOTIFY:
        m_eventType = MouseEventMoved;
        m_button = NoButton;
        m_clickCount = 0;
        break;
    default:
        ASSERT_NOT_REACHED();
    };

    if (motion->state & GDK_BUTTON1_MASK)
        m_button = LeftButton;
    else if (motion->state & GDK_BUTTON2_MASK)
        m_button = MiddleButton;
    else if (motion->state & GDK_BUTTON3_MASK)
        m_button = RightButton;
}
}
