

#include "config.h"
#include "PlatformMouseEvent.h"

#include <Message.h>
#include <SupportDefs.h>


namespace WebCore {

PlatformMouseEvent::PlatformMouseEvent(const BMessage* message)
    : m_timestamp(message->FindInt64("when"))
    , m_position(IntPoint(message->FindPoint("where")))
    , m_globalPosition(message->FindPoint("globalPosition"))
    , m_shiftKey(false)
    , m_ctrlKey(false)
    , m_altKey(false)
    , m_metaKey(false)
    , m_clickCount(message->FindInt32("clicks"))
{
    int32 buttons = message->FindInt32("buttons");
    switch (buttons) {
    case 1:
        m_button = LeftButton;
        break;
    case 2:
        m_button = RightButton;
        break;
    case 3:
        m_button = MiddleButton;
        break;
    default:
        m_button = NoButton;
        break;
    };

    switch (message->what) {
    case B_MOUSE_DOWN:
        m_eventType = MouseEventPressed;
        break;
    case B_MOUSE_UP:
        m_eventType = MouseEventReleased;
        m_button = LeftButton; // FIXME: Webcore wants to know the button released but we don't know.
        break;
    case B_MOUSE_MOVED:
        m_eventType = MouseEventMoved;
        break;
    default:
        m_eventType = MouseEventMoved;
        break;
    };
}

} // namespace WebCore

