

#ifndef SharedTimer_h
#define SharedTimer_h

#include <wtf/Noncopyable.h>

namespace WebCore {

    // Each thread has its own single instance of shared timer, which implements this interface.
    // This instance is shared by all timers in the thread.
    // Not intended to be used directly; use the Timer class instead.
    class SharedTimer : public Noncopyable {
    public:
        virtual ~SharedTimer() {}
        virtual void setFiredFunction(void (*)()) = 0;

        // The fire time is relative to the classic POSIX epoch of January 1, 1970,
        // as the result of currentTime() is.
        virtual void setFireTime(double) = 0;
        virtual void stop() = 0;
    };


    // Implemented by port (since it provides the run loop for the main thread).
    // FIXME: make ports implement MainThreadSharedTimer directly instead.
    void setSharedTimerFiredFunction(void (*)());
    void setSharedTimerFireTime(double);
    void stopSharedTimer();

    // Implementation of SharedTimer for the main thread.
    class MainThreadSharedTimer : public SharedTimer {
    public:
        virtual void setFiredFunction(void (*function)())
        {
            setSharedTimerFiredFunction(function);
        }
        
        virtual void setFireTime(double fireTime)
        {
            setSharedTimerFireTime(fireTime);
        }
        
        virtual void stop()
        {
            stopSharedTimer();
        }
    };

} // namespace WebCore

#endif // SharedTimer_h
