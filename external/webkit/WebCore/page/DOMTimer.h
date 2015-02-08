

#ifndef DOMTimer_h
#define DOMTimer_h

#include "ActiveDOMObject.h"
#include "Timer.h"
#include <wtf/OwnPtr.h>

namespace WebCore {

    class InspectorTimelineAgent;
    class ScheduledAction;

    class DOMTimer : public TimerBase, public ActiveDOMObject {
    public:
        virtual ~DOMTimer();
        // Creates a new timer owned by specified ScriptExecutionContext, starts it
        // and returns its Id.
        static int install(ScriptExecutionContext*, ScheduledAction*, int timeout, bool singleShot);
        static void removeById(ScriptExecutionContext*, int timeoutId);

        // ActiveDOMObject
        virtual bool hasPendingActivity() const;
        virtual void contextDestroyed();
        virtual void stop();
        virtual bool canSuspend() const;
        virtual void suspend();
        virtual void resume();

        // The lowest allowable timer setting (in seconds, 0.001 == 1 ms).
        // Default is 10ms.
        // Chromium uses a non-default timeout.
        static double minTimerInterval() { return s_minTimerInterval; }
        static void setMinTimerInterval(double value) { s_minTimerInterval = value; }

    private:
        DOMTimer(ScriptExecutionContext*, ScheduledAction*, int timeout, bool singleShot);
        virtual void fired();

        int m_timeoutId;
        int m_nestingLevel;
        OwnPtr<ScheduledAction> m_action;
        double m_nextFireInterval;
        double m_repeatInterval;
#if !ASSERT_DISABLED
        bool m_suspended;
#endif
        static double s_minTimerInterval;
    };

} // namespace WebCore

#endif // DOMTimer_h

