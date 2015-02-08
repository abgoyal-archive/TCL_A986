

#include "config.h"
#include "BeforeUnloadEvent.h"

#include "EventNames.h"

namespace WebCore {

BeforeUnloadEvent::BeforeUnloadEvent()
    : Event(eventNames().beforeunloadEvent, false, true)
{
}

bool BeforeUnloadEvent::storesResultAsString() const
{
    return true;
}

void BeforeUnloadEvent::storeResult(const String& s)
{
    m_result = s;
}

} // namespace WebCore
