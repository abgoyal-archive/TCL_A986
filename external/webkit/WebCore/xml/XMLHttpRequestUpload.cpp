

#include "config.h"
#include "XMLHttpRequestUpload.h"

#include "AtomicString.h"
#include "Event.h"
#include "EventException.h"
#include "EventNames.h"
#include "XMLHttpRequest.h"
#include "XMLHttpRequestProgressEvent.h"
#include <wtf/Assertions.h>

namespace WebCore {

XMLHttpRequestUpload::XMLHttpRequestUpload(XMLHttpRequest* xmlHttpRequest)
    : m_xmlHttpRequest(xmlHttpRequest)
{
}

ScriptExecutionContext* XMLHttpRequestUpload::scriptExecutionContext() const
{
    XMLHttpRequest* xmlHttpRequest = associatedXMLHttpRequest();
    if (!xmlHttpRequest)
        return 0;
    return xmlHttpRequest->scriptExecutionContext();
}

EventTargetData* XMLHttpRequestUpload::eventTargetData()
{
    return &m_eventTargetData;
}

EventTargetData* XMLHttpRequestUpload::ensureEventTargetData()
{
    return &m_eventTargetData;
}

} // namespace WebCore
