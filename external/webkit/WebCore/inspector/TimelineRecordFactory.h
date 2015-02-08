
 
#ifndef TimelineRecordFactory_h
#define TimelineRecordFactory_h

#include "PlatformString.h"

namespace WebCore {

    class Event;
    class InspectorFrontend;
    class IntRect;
    class ResourceRequest;
    class ResourceResponse;
    class ScriptObject;

    class TimelineRecordFactory {
    public:
        static ScriptObject createGenericRecord(InspectorFrontend*, double startTime);

        static ScriptObject createEventDispatchData(InspectorFrontend*, const Event&);

        static ScriptObject createGenericTimerData(InspectorFrontend*, int timerId);

        static ScriptObject createTimerInstallData(InspectorFrontend*, int timerId, int timeout, bool singleShot);

        static ScriptObject createXHRReadyStateChangeData(InspectorFrontend*, const String& url, int readyState);

        static ScriptObject createXHRLoadData(InspectorFrontend*, const String& url);
        
        static ScriptObject createEvaluateScriptData(InspectorFrontend*, const String&, double lineNumber);
        
        static ScriptObject createMarkTimelineData(InspectorFrontend*, const String&);

        static ScriptObject createResourceSendRequestData(InspectorFrontend*, unsigned long identifier,
            bool isMainResource, const ResourceRequest&);

        static ScriptObject createResourceReceiveResponseData(InspectorFrontend*, unsigned long identifier, const ResourceResponse&);

        static ScriptObject createResourceFinishData(InspectorFrontend*, unsigned long identifier, bool didFail);

        static ScriptObject createPaintData(InspectorFrontend*, const IntRect&);

        static ScriptObject createParseHTMLData(InspectorFrontend*, unsigned int length, unsigned int startLine);

    private:
        TimelineRecordFactory() { }
    };

} // namespace WebCore

#endif // !defined(TimelineRecordFactory_h)
