

#include "config.h"
#include "PlatformWheelEvent.h"
#include "Scrollbar.h"

#include <wx/defs.h>
#include <wx/event.h>

namespace WebCore {

PlatformWheelEvent::PlatformWheelEvent(const wxMouseEvent& event, const wxPoint& globalPoint)
    : m_position(event.GetPosition())
    , m_globalPosition(globalPoint)
    , m_granularity(ScrollByPixelWheelEvent)
    , m_shiftKey(event.ShiftDown())
    , m_ctrlKey(event.ControlDown())
    , m_altKey(event.AltDown())
    , m_metaKey(event.MetaDown()) // FIXME: We'll have to test other browsers
    , m_deltaX(0) // wx doesn't support horizontal mouse wheel scrolling
    , m_deltaY(event.GetWheelRotation() / event.GetWheelDelta())
    , m_wheelTicksX(m_deltaX)
    , m_wheelTicksY(m_deltaY)
    , m_isAccepted(false)
{
    // FIXME: retrieve the user setting for the number of lines to scroll on each wheel event
    m_deltaY *= static_cast<float>(Scrollbar::pixelsPerLineStep());
}

}
