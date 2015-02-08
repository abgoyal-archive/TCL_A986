

#ifndef ThreadTimer_h
#define ThreadTimer_h

#include <wtf/Noncopyable.h>
#include <wtf/HashSet.h>
#include <wtf/Vector.h>

namespace WebCore {

    class SharedTimer;
    class TimerBase;

    // A collection of timers per thread. Kept in ThreadGlobalData.
    class ThreadTimers : public Noncopyable {
    public:
        ThreadTimers();

        // On a thread different then main, we should set the thread's instance of the SharedTimer.
        void setSharedTimer(SharedTimer*);

        Vector<TimerBase*>& timerHeap() { return m_timerHeap; }

        void updateSharedTimer();
        void fireTimersInNestedEventLoop();

    private:
        static void sharedTimerFired();

        void sharedTimerFiredInternal();
        void fireTimersInNestedEventLoopInternal();

        Vector<TimerBase*> m_timerHeap;
        SharedTimer* m_sharedTimer; // External object, can be a run loop on a worker thread. Normally set/reset by worker thread.
        bool m_firingTimers; // Reentrancy guard.
    };

}

#endif
