

#include "config.h"
#include "RunLoopTimer.h"

#if PLATFORM(MAC) && HAVE(RUNLOOP_TIMER)

namespace WebCore {

RunLoopTimerBase::~RunLoopTimerBase()
{
    stop();
}

static void timerFired(CFRunLoopTimerRef, void* context)
{
    RunLoopTimerBase* timer = static_cast<RunLoopTimerBase*>(context);
    timer->fired();
}

void RunLoopTimerBase::start(double nextFireInterval, double repeatInterval)
{
    if (m_timer)
        CFRunLoopTimerInvalidate(m_timer.get());
    CFRunLoopTimerContext context = { 0, this, 0, 0, 0 };
    m_timer.adoptCF(CFRunLoopTimerCreate(0, CFAbsoluteTimeGetCurrent() + nextFireInterval, repeatInterval, 0, 0, timerFired, &context));
}

void RunLoopTimerBase::schedule(const SchedulePair* schedulePair)
{
    ASSERT_ARG(schedulePair, schedulePair);
    ASSERT_WITH_MESSAGE(m_timer, "Timer must have one of the start functions called before calling schedule().");
    CFRunLoopAddTimer(schedulePair->runLoop(), m_timer.get(), schedulePair->mode());
}

void RunLoopTimerBase::schedule(const SchedulePairHashSet& schedulePairs)
{
    SchedulePairHashSet::const_iterator end = schedulePairs.end();
    for (SchedulePairHashSet::const_iterator it = schedulePairs.begin(); it != end; ++it)
        schedule((*it).get());
}

void RunLoopTimerBase::stop()
{
    if (!m_timer)
        return;
    CFRunLoopTimerInvalidate(m_timer.get());
    m_timer = 0;
}

bool RunLoopTimerBase::isActive() const
{
    return m_timer && CFRunLoopTimerIsValid(m_timer.get());
}

} // namespace WebCore

#endif // PLATFORM(MAC) && HAVE(RUNLOOP_TIMER)
