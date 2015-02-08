

#include "config.h"
#include "WebKitTransitionEvent.h"

#include "EventNames.h"

namespace WebCore {
        
WebKitTransitionEvent::WebKitTransitionEvent()
    : m_propertyName()
    , m_elapsedTime(0.0)
{
}

WebKitTransitionEvent::WebKitTransitionEvent(const AtomicString& type, const String& propertyName, double elapsedTime)
    : Event(type, true, true)
    , m_propertyName(propertyName)
    , m_elapsedTime(elapsedTime)
{
}

WebKitTransitionEvent::~WebKitTransitionEvent()
{
}

void WebKitTransitionEvent::initWebKitTransitionEvent(const AtomicString& type, 
                                            bool canBubbleArg,
                                            bool cancelableArg,
                                            const String& propertyName,
                                            double elapsedTime)
{
    if (dispatched())
        return;
    
    initEvent(type, canBubbleArg, cancelableArg);
    
    m_propertyName = propertyName;
    m_elapsedTime = elapsedTime;
}

const String& WebKitTransitionEvent::propertyName() const
{
    return m_propertyName;
}

double WebKitTransitionEvent::elapsedTime() const
{
    return m_elapsedTime;
}

} // namespace WebCore
