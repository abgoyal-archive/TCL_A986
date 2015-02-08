

#ifndef InspectorTimelineAgent_h
#define InspectorTimelineAgent_h

#if ENABLE(INSPECTOR)

#include "Document.h"
#include "ScriptExecutionContext.h"
#include "ScriptObject.h"
#include "ScriptArray.h"
#include <wtf/Vector.h>

namespace WebCore {
    class Event;
    class InspectorFrontend;
    class IntRect;
    class ResourceRequest;
    class ResourceResponse;

    // Must be kept in sync with TimelineAgent.js
    enum TimelineRecordType {
        EventDispatchTimelineRecordType = 0,
        LayoutTimelineRecordType = 1,
        RecalculateStylesTimelineRecordType = 2,
        PaintTimelineRecordType = 3,
        ParseHTMLTimelineRecordType = 4,
        TimerInstallTimelineRecordType = 5,
        TimerRemoveTimelineRecordType = 6,
        TimerFireTimelineRecordType = 7,
        XHRReadyStateChangeRecordType = 8,
        XHRLoadRecordType = 9,
        EvaluateScriptTimelineRecordType = 10,
        MarkTimelineRecordType = 11,
        ResourceSendRequestTimelineRecordType = 12,
        ResourceReceiveResponseTimelineRecordType = 13,
        ResourceFinishTimelineRecordType = 14,
    };

    class InspectorTimelineAgent : public Noncopyable {
    public:
        InspectorTimelineAgent(InspectorFrontend* frontend);
        ~InspectorTimelineAgent();

        void reset();
        void resetFrontendProxyObject(InspectorFrontend*);

        // Methods called from WebCore.
        void willDispatchEvent(const Event&);
        void didDispatchEvent();

        void willLayout();
        void didLayout();

        void willRecalculateStyle();
        void didRecalculateStyle();

        void willPaint(const IntRect&);
        void didPaint();

        void willWriteHTML(unsigned int length, unsigned int startLine);
        void didWriteHTML(unsigned int endLine);
        
        void didInstallTimer(int timerId, int timeout, bool singleShot);
        void didRemoveTimer(int timerId);
        void willFireTimer(int timerId);
        void didFireTimer();

        void willChangeXHRReadyState(const String&, int);
        void didChangeXHRReadyState();
        void willLoadXHR(const String&);
        void didLoadXHR();

        void willEvaluateScript(const String&, int);
        void didEvaluateScript();

        void didMarkTimeline(const String&);

        void willSendResourceRequest(unsigned long, bool isMainResource, const ResourceRequest&);
        void didReceiveResourceResponse(unsigned long, const ResourceResponse&);
        void didFinishLoadingResource(unsigned long, bool didFail);

        static InspectorTimelineAgent* retrieve(ScriptExecutionContext*);
    private:
        struct TimelineRecordEntry {
            TimelineRecordEntry(ScriptObject record, ScriptObject data, ScriptArray children, TimelineRecordType type) : record(record), data(data), children(children), type(type) { }
            ScriptObject record;
            ScriptObject data;
            ScriptArray children;
            TimelineRecordType type;
        };
        
        void pushCurrentRecord(ScriptObject, TimelineRecordType);
        
        static double currentTimeInMilliseconds();

        void didCompleteCurrentRecord(TimelineRecordType);

        void addRecordToTimeline(ScriptObject, TimelineRecordType);

        InspectorFrontend* m_frontend;
        
        Vector< TimelineRecordEntry > m_recordStack;
    };

inline InspectorTimelineAgent* InspectorTimelineAgent::retrieve(ScriptExecutionContext* context)
{
    if (context && context->isDocument())
        return static_cast<Document*>(context)->inspectorTimelineAgent();
    return 0;
}

} // namespace WebCore

#endif // !ENABLE(INSPECTOR)
#endif // !defined(InspectorTimelineAgent_h)
