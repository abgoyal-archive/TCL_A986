

#include "config.h"
#include "PopStateEvent.h"

#include "EventNames.h"

namespace WebCore {

PopStateEvent::PopStateEvent(PassRefPtr<SerializedScriptValue> stateObject)
    : Event(eventNames().popstateEvent, false, true)
    , m_stateObject(stateObject)
{
}
    
void PopStateEvent::initPopStateEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<SerializedScriptValue> stateObject)
{
    if (dispatched())
        return;
    
    initEvent(type, canBubble, cancelable);

    m_stateObject = stateObject;
}

} // namespace WebCore
