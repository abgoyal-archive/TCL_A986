

#ifndef Timer_h
#define Timer_h

#include <wtf/Noncopyable.h>
#include <wtf/Threading.h>

namespace WebCore {

// Time intervals are all in seconds.

class TimerHeapElement;

class TimerBase : public Noncopyable {
public:
    TimerBase();
    virtual ~TimerBase();

    void start(double nextFireInterval, double repeatInterval);

    void startRepeating(double repeatInterval) { start(repeatInterval, repeatInterval); }
    void startOneShot(double interval) { start(interval, 0); }

    void stop();
    bool isActive() const;

    double nextFireInterval() const;
    double repeatInterval() const { return m_repeatInterval; }

    void augmentRepeatInterval(double delta) { setNextFireTime(m_nextFireTime + delta); m_repeatInterval += delta; }

    static void fireTimersInNestedEventLoop();

private:
    virtual void fired() = 0;

    void checkConsistency() const;
    void checkHeapIndex() const;

    void setNextFireTime(double);

    bool inHeap() const { return m_heapIndex != -1; }

    void heapDecreaseKey();
    void heapDelete();
    void heapDeleteMin();
    void heapIncreaseKey();
    void heapInsert();
    void heapPop();
    void heapPopMin();

    double m_nextFireTime; // 0 if inactive
    double m_repeatInterval; // 0 if not repeating
    int m_heapIndex; // -1 if not in heap
    unsigned m_heapInsertionOrder; // Used to keep order among equal-fire-time timers

#ifndef NDEBUG
    ThreadIdentifier m_thread;
#endif

    friend class TimerHeapElement;
    friend class ThreadTimers;
    friend bool operator<(const TimerHeapElement&, const TimerHeapElement&);
};

template <typename TimerFiredClass> class Timer : public TimerBase {
public:
    typedef void (TimerFiredClass::*TimerFiredFunction)(Timer*);

    Timer(TimerFiredClass* o, TimerFiredFunction f)
        : m_object(o), m_function(f) { }

private:
    virtual void fired() { (m_object->*m_function)(this); }

    TimerFiredClass* m_object;
    TimerFiredFunction m_function;
};

inline bool TimerBase::isActive() const
{
    ASSERT(m_thread == currentThread());
    return m_nextFireTime;
}

}

#endif
