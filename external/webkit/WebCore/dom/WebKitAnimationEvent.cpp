

#include "config.h"
#include "WebKitAnimationEvent.h"

#include "EventNames.h"

namespace WebCore {

WebKitAnimationEvent::WebKitAnimationEvent()
    : m_elapsedTime(0.0)
{
}

WebKitAnimationEvent::WebKitAnimationEvent(const AtomicString& type, const String& animationName, double elapsedTime)
    : Event(type, true, true)
    , m_animationName(animationName)
    , m_elapsedTime(elapsedTime)
{
}

WebKitAnimationEvent::~WebKitAnimationEvent()
{
}

void WebKitAnimationEvent::initWebKitAnimationEvent(const AtomicString& type, 
                                        bool canBubbleArg,
                                        bool cancelableArg,
                                        const String& animationName,
                                        double elapsedTime)
{
    if (dispatched())
        return;
    
    initEvent(type, canBubbleArg, cancelableArg);
    
    m_animationName = animationName;
    m_elapsedTime = elapsedTime;
}

const String& WebKitAnimationEvent::animationName() const
{
    return m_animationName;
}

double WebKitAnimationEvent::elapsedTime() const
{
    return m_elapsedTime;
}

} // namespace WebCore
