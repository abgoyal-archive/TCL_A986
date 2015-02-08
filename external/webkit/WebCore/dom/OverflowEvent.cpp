

#include "config.h"
#include "OverflowEvent.h"

#include "EventNames.h"

namespace WebCore {
        
OverflowEvent::OverflowEvent()
    : Event(eventNames().overflowchangedEvent, false, false)
    , m_orient(VERTICAL)
    , m_horizontalOverflow(false)
    , m_verticalOverflow(false)
{
}

OverflowEvent::OverflowEvent(bool horizontalOverflowChanged, bool horizontalOverflow, bool verticalOverflowChanged, bool verticalOverflow)
    : Event(eventNames().overflowchangedEvent, false, false)
    , m_horizontalOverflow(horizontalOverflow)
    , m_verticalOverflow(verticalOverflow)
{
    ASSERT(horizontalOverflowChanged || verticalOverflowChanged);

    if (horizontalOverflowChanged && verticalOverflowChanged)
        m_orient = BOTH;
    else if (horizontalOverflowChanged)
        m_orient = HORIZONTAL;
    else
        m_orient = VERTICAL;
}

bool OverflowEvent::isOverflowEvent() const
{
    return true;
}

void OverflowEvent::initOverflowEvent(unsigned short orient, bool horizontalOverflow, bool verticalOverflow)
{
    if (dispatched())
        return;
    
    m_orient = orient;
    m_horizontalOverflow = horizontalOverflow;
    m_verticalOverflow = verticalOverflow;
}

}
