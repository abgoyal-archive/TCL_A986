

#include "config.h"
#include "TimelineRecordFactory.h"

#if ENABLE(INSPECTOR)

#include "Event.h"
#include "InspectorFrontend.h"
#include "IntRect.h"
#include "ResourceRequest.h"
#include "ResourceResponse.h"
#include "ScriptArray.h"
#include "ScriptObject.h"

namespace WebCore {

ScriptObject TimelineRecordFactory::createGenericRecord(InspectorFrontend* frontend, double startTime)
{
    ScriptObject record = frontend->newScriptObject();
    record.set("startTime", startTime);
    return record;
}

ScriptObject TimelineRecordFactory::createEventDispatchData(InspectorFrontend* frontend, const Event& event)
{
    ScriptObject data = frontend->newScriptObject();
    data.set("type", event.type().string());
    return data;
}

ScriptObject TimelineRecordFactory::createGenericTimerData(InspectorFrontend* frontend, int timerId)
{
    ScriptObject data = frontend->newScriptObject();
    data.set("timerId", timerId);
    return data;
}

ScriptObject TimelineRecordFactory::createTimerInstallData(InspectorFrontend* frontend, int timerId, int timeout, bool singleShot)
{
    ScriptObject data = frontend->newScriptObject();
    data.set("timerId", timerId);
    data.set("timeout", timeout);
    data.set("singleShot", singleShot);
    return data;
}

ScriptObject TimelineRecordFactory::createXHRReadyStateChangeData(InspectorFrontend* frontend, const String& url, int readyState)
{
    ScriptObject data = frontend->newScriptObject();
    data.set("url", url);
    data.set("readyState", readyState);
    return data;
}

ScriptObject TimelineRecordFactory::createXHRLoadData(InspectorFrontend* frontend, const String& url)
{
    ScriptObject data = frontend->newScriptObject();
    data.set("url", url);
    return data;
}

ScriptObject TimelineRecordFactory::createEvaluateScriptData(InspectorFrontend* frontend, const String& url, double lineNumber) 
{
    ScriptObject data = frontend->newScriptObject();
    data.set("url", url);
    data.set("lineNumber", lineNumber);
    return data;
}

ScriptObject TimelineRecordFactory::createMarkTimelineData(InspectorFrontend* frontend, const String& message) 
{
    ScriptObject data = frontend->newScriptObject();
    data.set("message", message);
    return data;
}


ScriptObject TimelineRecordFactory::createResourceSendRequestData(InspectorFrontend* frontend, unsigned long identifier, bool isMainResource, const ResourceRequest& request)
{
    ScriptObject data = frontend->newScriptObject();
    data.set("identifier", identifier);
    data.set("url", request.url().string());
    data.set("requestMethod", request.httpMethod());
    data.set("isMainResource", isMainResource);
    return data;
}

ScriptObject TimelineRecordFactory::createResourceReceiveResponseData(InspectorFrontend* frontend, unsigned long identifier, const ResourceResponse& response)
{
    ScriptObject data = frontend->newScriptObject();
    data.set("identifier", identifier);
    data.set("statusCode", response.httpStatusCode());
    data.set("mimeType", response.mimeType());
    data.set("expectedContentLength", response.expectedContentLength());
    return data;
}

ScriptObject TimelineRecordFactory::createResourceFinishData(InspectorFrontend* frontend, unsigned long identifier, bool didFail)
{
    ScriptObject data = frontend->newScriptObject();
    data.set("identifier", identifier);
    data.set("didFail", didFail);
    return data;
}

ScriptObject TimelineRecordFactory::createPaintData(InspectorFrontend* frontend, const IntRect& rect)
{
    ScriptObject data = frontend->newScriptObject();
    data.set("x", rect.x());
    data.set("y", rect.y());
    data.set("width", rect.width());
    data.set("height", rect.height());
    return data;
}

ScriptObject TimelineRecordFactory::createParseHTMLData(InspectorFrontend* frontend, unsigned int length, unsigned int startLine)
{
    ScriptObject data = frontend->newScriptObject();
    data.set("length", length);
    data.set("startLine", startLine);
    return data;
}

} // namespace WebCore

#endif // ENABLE(INSPECTOR)
