

#include "config.h"
#include "ThreadTimers.h"

#include "SharedTimer.h"
#include "ThreadGlobalData.h"
#include "Timer.h"
#include <wtf/CurrentTime.h>

namespace WebCore {

// Fire timers for this length of time, and then quit to let the run loop process user input events.
// 100ms is about a perceptable delay in UI, so use a half of that as a threshold.
// This is to prevent UI freeze when there are too many timers or machine performance is low.
static const double maxDurationOfFiringTimers = 0.050;

// Timers are created, started and fired on the same thread, and each thread has its own ThreadTimers
// copy to keep the heap and a set of currently firing timers.

static MainThreadSharedTimer* mainThreadSharedTimer()
{
    static MainThreadSharedTimer* timer = new MainThreadSharedTimer;
    return timer;
}

ThreadTimers::ThreadTimers()
    : m_sharedTimer(0)
    , m_firingTimers(false)
{
    if (isMainThread())
        setSharedTimer(mainThreadSharedTimer());
}

// A worker thread may initialize SharedTimer after some timers are created.
// Also, SharedTimer can be replaced with 0 before all timers are destroyed.
void ThreadTimers::setSharedTimer(SharedTimer* sharedTimer)
{
    if (m_sharedTimer) {
        m_sharedTimer->setFiredFunction(0);
        m_sharedTimer->stop();
    }
    
    m_sharedTimer = sharedTimer;
    
    if (sharedTimer) {
        m_sharedTimer->setFiredFunction(ThreadTimers::sharedTimerFired);
        updateSharedTimer();
    }
}

void ThreadTimers::updateSharedTimer()
{
    if (!m_sharedTimer)
        return;
        
    if (m_firingTimers || m_timerHeap.isEmpty())
        m_sharedTimer->stop();
    else
        m_sharedTimer->setFireTime(m_timerHeap.first()->m_nextFireTime);
}

void ThreadTimers::sharedTimerFired()
{
    // Redirect to non-static method.
    threadGlobalData().threadTimers().sharedTimerFiredInternal();
}

void ThreadTimers::sharedTimerFiredInternal()
{
    // Do a re-entrancy check.
    if (m_firingTimers)
        return;
    m_firingTimers = true;

    double fireTime = currentTime();
    double timeToQuit = fireTime + maxDurationOfFiringTimers;

    while (!m_timerHeap.isEmpty() && m_timerHeap.first()->m_nextFireTime <= fireTime) {
        TimerBase* timer = m_timerHeap.first();
        timer->m_nextFireTime = 0;
        timer->heapDeleteMin();

        double interval = timer->repeatInterval();
        timer->setNextFireTime(interval ? fireTime + interval : 0);

        // Once the timer has been fired, it may be deleted, so do nothing else with it after this point.
        timer->fired();

        // Catch the case where the timer asked timers to fire in a nested event loop, or we are over time limit.
        if (!m_firingTimers || timeToQuit < currentTime())
            break;
    }

    m_firingTimers = false;

    updateSharedTimer();
}

void ThreadTimers::fireTimersInNestedEventLoop()
{
    // Reset the reentrancy guard so the timers can fire again.
    m_firingTimers = false;
    updateSharedTimer();
}

} // namespace WebCore

