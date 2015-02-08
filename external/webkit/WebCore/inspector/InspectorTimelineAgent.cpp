

#include "config.h"
#include "InspectorTimelineAgent.h"

#if ENABLE(INSPECTOR)

#include "Event.h"
#include "InspectorFrontend.h"
#include "IntRect.h"
#include "ResourceRequest.h"
#include "ResourceResponse.h"
#include "TimelineRecordFactory.h"

#include <wtf/CurrentTime.h>

namespace WebCore {

InspectorTimelineAgent::InspectorTimelineAgent(InspectorFrontend* frontend)
    : m_frontend(frontend)
{
    ASSERT(m_frontend);
}

InspectorTimelineAgent::~InspectorTimelineAgent()
{
}

void InspectorTimelineAgent::willDispatchEvent(const Event& event)
{
    pushCurrentRecord(TimelineRecordFactory::createEventDispatchData(m_frontend, event),
        EventDispatchTimelineRecordType);
}

void InspectorTimelineAgent::didDispatchEvent()
{
    didCompleteCurrentRecord(EventDispatchTimelineRecordType);
}

void InspectorTimelineAgent::willLayout()
{
    pushCurrentRecord(m_frontend->newScriptObject(), LayoutTimelineRecordType);
}

void InspectorTimelineAgent::didLayout()
{
    didCompleteCurrentRecord(LayoutTimelineRecordType);
}

void InspectorTimelineAgent::willRecalculateStyle()
{
    pushCurrentRecord(m_frontend->newScriptObject(), RecalculateStylesTimelineRecordType);
}

void InspectorTimelineAgent::didRecalculateStyle()
{
    didCompleteCurrentRecord(RecalculateStylesTimelineRecordType);
}

void InspectorTimelineAgent::willPaint(const IntRect& rect)
{
    pushCurrentRecord(TimelineRecordFactory::createPaintData(m_frontend, rect), PaintTimelineRecordType);
}

void InspectorTimelineAgent::didPaint()
{
    didCompleteCurrentRecord(PaintTimelineRecordType);
}

void InspectorTimelineAgent::willWriteHTML(unsigned int length, unsigned int startLine)
{
    pushCurrentRecord(TimelineRecordFactory::createParseHTMLData(m_frontend, length, startLine), ParseHTMLTimelineRecordType);
}

void InspectorTimelineAgent::didWriteHTML(unsigned int endLine)
{
    if (!m_recordStack.isEmpty()) {
        TimelineRecordEntry entry = m_recordStack.last();
        entry.data.set("endLine", endLine);
        didCompleteCurrentRecord(ParseHTMLTimelineRecordType);
    }
}
   
void InspectorTimelineAgent::didInstallTimer(int timerId, int timeout, bool singleShot)
{
    ScriptObject record = TimelineRecordFactory::createGenericRecord(m_frontend, currentTimeInMilliseconds());
    record.set("data", TimelineRecordFactory::createTimerInstallData(m_frontend, timerId, timeout, singleShot));
    addRecordToTimeline(record, TimerInstallTimelineRecordType);
}

void InspectorTimelineAgent::didRemoveTimer(int timerId)
{
    ScriptObject record = TimelineRecordFactory::createGenericRecord(m_frontend, currentTimeInMilliseconds());
    record.set("data", TimelineRecordFactory::createGenericTimerData(m_frontend, timerId));
    addRecordToTimeline(record, TimerRemoveTimelineRecordType);
}

void InspectorTimelineAgent::willFireTimer(int timerId)
{
    pushCurrentRecord(TimelineRecordFactory::createGenericTimerData(m_frontend, timerId), TimerFireTimelineRecordType); 
}

void InspectorTimelineAgent::didFireTimer()
{
    didCompleteCurrentRecord(TimerFireTimelineRecordType);
}

void InspectorTimelineAgent::willChangeXHRReadyState(const String& url, int readyState)
{
    pushCurrentRecord(TimelineRecordFactory::createXHRReadyStateChangeData(m_frontend, url, readyState), XHRReadyStateChangeRecordType);
}

void InspectorTimelineAgent::didChangeXHRReadyState()
{
    didCompleteCurrentRecord(XHRReadyStateChangeRecordType);
}

void InspectorTimelineAgent::willLoadXHR(const String& url) 
{
    pushCurrentRecord(TimelineRecordFactory::createXHRLoadData(m_frontend, url), XHRLoadRecordType);
}

void InspectorTimelineAgent::didLoadXHR()
{
    didCompleteCurrentRecord(XHRLoadRecordType);
}

void InspectorTimelineAgent::willEvaluateScript(const String& url, int lineNumber)
{
    pushCurrentRecord(TimelineRecordFactory::createEvaluateScriptData(m_frontend, url, lineNumber), EvaluateScriptTimelineRecordType);
}
    
void InspectorTimelineAgent::didEvaluateScript()
{
    didCompleteCurrentRecord(EvaluateScriptTimelineRecordType);
}

void InspectorTimelineAgent::willSendResourceRequest(unsigned long identifier, bool isMainResource,
    const ResourceRequest& request)
{
    ScriptObject record = TimelineRecordFactory::createGenericRecord(m_frontend, currentTimeInMilliseconds());
    record.set("data", TimelineRecordFactory::createResourceSendRequestData(m_frontend, identifier, isMainResource, request));
    record.set("type", ResourceSendRequestTimelineRecordType);
    m_frontend->addRecordToTimeline(record);
}

void InspectorTimelineAgent::didReceiveResourceResponse(unsigned long identifier, const ResourceResponse& response)
{
    ScriptObject record = TimelineRecordFactory::createGenericRecord(m_frontend, currentTimeInMilliseconds());
    record.set("data", TimelineRecordFactory::createResourceReceiveResponseData(m_frontend, identifier, response));
    record.set("type", ResourceReceiveResponseTimelineRecordType);
    m_frontend->addRecordToTimeline(record);
}

void InspectorTimelineAgent::didFinishLoadingResource(unsigned long identifier, bool didFail)
{
    ScriptObject record = TimelineRecordFactory::createGenericRecord(m_frontend, currentTimeInMilliseconds());
    record.set("data", TimelineRecordFactory::createResourceFinishData(m_frontend, identifier, didFail));
    record.set("type", ResourceFinishTimelineRecordType);
    m_frontend->addRecordToTimeline(record);
}

void InspectorTimelineAgent::didMarkTimeline(const String& message)
{
    ScriptObject record = TimelineRecordFactory::createGenericRecord(m_frontend, currentTimeInMilliseconds());
    record.set("data", TimelineRecordFactory::createMarkTimelineData(m_frontend, message));
    addRecordToTimeline(record, MarkTimelineRecordType);
}

void InspectorTimelineAgent::reset()
{
    m_recordStack.clear();
}

void InspectorTimelineAgent::resetFrontendProxyObject(InspectorFrontend* frontend)
{
    ASSERT(frontend);
    reset();
    m_frontend = frontend;
}

void InspectorTimelineAgent::addRecordToTimeline(ScriptObject record, TimelineRecordType type)
{
    record.set("type", type);
    if (m_recordStack.isEmpty())
        m_frontend->addRecordToTimeline(record);
    else {
        TimelineRecordEntry parent = m_recordStack.last();
        parent.children.set(parent.children.length(), record);
    }
}

void InspectorTimelineAgent::didCompleteCurrentRecord(TimelineRecordType type)
{
    // An empty stack could merely mean that the timeline agent was turned on in the middle of
    // an event.  Don't treat as an error.
    if (!m_recordStack.isEmpty()) {
        TimelineRecordEntry entry = m_recordStack.last();
        m_recordStack.removeLast();
        ASSERT(entry.type == type);
        entry.record.set("data", entry.data);
        entry.record.set("children", entry.children);
        entry.record.set("endTime", currentTimeInMilliseconds());
        addRecordToTimeline(entry.record, type);
    }
}

double InspectorTimelineAgent::currentTimeInMilliseconds()
{
    return currentTime() * 1000.0;
}

void InspectorTimelineAgent::pushCurrentRecord(ScriptObject data, TimelineRecordType type)
{
    m_recordStack.append(TimelineRecordEntry(TimelineRecordFactory::createGenericRecord(m_frontend, currentTimeInMilliseconds()), data, m_frontend->newScriptArray(), type));
}

} // namespace WebCore

#endif // ENABLE(INSPECTOR)
