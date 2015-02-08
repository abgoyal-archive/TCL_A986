

#include "config.h"
#include "BeforeTextInsertedEvent.h"

#include "EventNames.h"

namespace WebCore {

BeforeTextInsertedEvent::BeforeTextInsertedEvent(const String& text)
    : Event(eventNames().webkitBeforeTextInsertedEvent, false, true), m_text(text)
{
}

}
