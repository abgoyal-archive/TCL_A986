

#ifndef TIMER_CLIENT_H
#define TIMER_CLIENT_H

namespace android {

    class TimerClient
    {
    public:
        virtual ~TimerClient() {}
        virtual void setSharedTimerCallback(void(*f)()) = 0;
        virtual void setSharedTimer(long long timemillis) = 0;
        virtual void stopSharedTimer() = 0;
        virtual void signalServiceFuncPtrQueue() = 0;
    };

}
#endif
