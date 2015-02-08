

#include "config.h"
#include "PlatformWheelEvent.h"

#include "Scrollbar.h"

#include <Message.h>
#include <Point.h>


namespace WebCore {

PlatformWheelEvent::PlatformWheelEvent(BMessage* message)
    : m_granularity(ScrollByPixelWheelEvent)
    , m_shiftKey(false)
    , m_ctrlKey(false)
    , m_altKey(false)
    , m_metaKey(false)
    , m_isAccepted(false)
{
    m_position = IntPoint(message->FindPoint("position"));
    m_globalPosition = IntPoint(message->FindPoint("global_position"));

    m_deltaX = message->FindFloat("be:wheel_delta_x");
    m_deltaY = message->FindFloat("be:wheel_delta_y");

    m_wheelTicksX = m_deltaX;
    m_wheelTicksY = m_deltaY;

    m_deltaX *= -Scrollbar::pixelsPerLineStep();
    m_deltaY *= -Scrollbar::pixelsPerLineStep();
}

} // namespace WebCore

