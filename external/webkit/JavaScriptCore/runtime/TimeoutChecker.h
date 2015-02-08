

#ifndef TimeoutChecker_h
#define TimeoutChecker_h

#include <wtf/Assertions.h>

#ifdef ANDROID_INSTRUMENT
#include "TimeCounter.h"
#endif

namespace JSC {

    class ExecState;

    class TimeoutChecker {
    public:
        TimeoutChecker();

        void setTimeoutInterval(unsigned timeoutInterval) { m_timeoutInterval = timeoutInterval; }
        
        unsigned ticksUntilNextCheck() { return m_ticksUntilNextCheck; }
        
        void start()
        {
            if (!m_startCount)
                reset();
#ifdef ANDROID_INSTRUMENT
            if (!m_startCount)
                android::TimeCounter::start(android::TimeCounter::JavaScriptTimeCounter);
#endif
            ++m_startCount;
        }

        void stop()
        {
            ASSERT(m_startCount);
            --m_startCount;
#ifdef ANDROID_INSTRUMENT
            if (!m_startCount)
                android::TimeCounter::record(android::TimeCounter::JavaScriptTimeCounter, __FUNCTION__);
#endif
        }

        void reset();

        bool didTimeOut(ExecState*);

    private:
        unsigned m_timeoutInterval;
        unsigned m_timeAtLastCheck;
        unsigned m_timeExecuting;
        unsigned m_startCount;
        unsigned m_ticksUntilNextCheck;
    };

} // namespace JSC

#endif // TimeoutChecker_h
