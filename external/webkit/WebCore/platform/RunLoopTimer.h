

#ifndef RunLoopTimer_h
#define RunLoopTimer_h

#include "SchedulePair.h"
#include <wtf/Noncopyable.h>
#include <wtf/RetainPtr.h>

namespace WebCore {

// Time intervals are all in seconds.

class RunLoopTimerBase : public Noncopyable {
public:
    virtual ~RunLoopTimerBase();

    void schedule(const SchedulePair*);
    void schedule(const SchedulePairHashSet&);

    void start(double nextFireInterval, double repeatInterval);

    void startRepeating(double repeatInterval) { start(repeatInterval, repeatInterval); }
    void startOneShot(double interval) { start(interval, 0); }

    void stop();
    bool isActive() const;

    virtual void fired() = 0;

private:
#if PLATFORM(CF)
    RetainPtr<CFRunLoopTimerRef> m_timer;
#endif
};

template <typename TimerFiredClass> class RunLoopTimer : public RunLoopTimerBase {
public:
    typedef void (TimerFiredClass::*TimerFiredFunction)(RunLoopTimer*);

    RunLoopTimer(TimerFiredClass* o, TimerFiredFunction f)
        : m_object(o), m_function(f) { }

    virtual void fired() { (m_object->*m_function)(this); }

private:
    TimerFiredClass* m_object;
    TimerFiredFunction m_function;
};

} // namespace WebCore

#endif
