

#include "config.h"

#if ENABLE(WORKERS)

#include "ErrorEvent.h"

#include "EventNames.h"

namespace WebCore {

ErrorEvent::ErrorEvent()
{
}

ErrorEvent::ErrorEvent(const String& message, const String& fileName, unsigned lineNumber)
    : Event(eventNames().errorEvent, false, true)
    , m_message(message)
    , m_fileName(fileName)
    , m_lineNumber(lineNumber)
{
}

ErrorEvent::~ErrorEvent()
{
}

void ErrorEvent::initErrorEvent(const AtomicString& type, bool canBubble, bool cancelable, const String& message, const String& fileName, unsigned lineNumber)
{
    if (dispatched())
        return;
        
    initEvent(type, canBubble, cancelable);
    
    m_message = message;
    m_fileName = fileName;
    m_lineNumber = lineNumber;
}

bool ErrorEvent::isErrorEvent() const 
{
    return true;
}

} // namespace WebCore

#endif // ENABLE(WORKERS)
