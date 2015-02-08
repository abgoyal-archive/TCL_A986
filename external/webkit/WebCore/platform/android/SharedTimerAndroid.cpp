

#include "config.h"
#include "SharedTimer.h"

#define LOG_TAG "Timers"

#include <TimerClient.h>
#include <JavaSharedClient.h>
#include <utils/Log.h>
#include <wtf/CurrentTime.h>

using namespace android;

namespace WebCore {

// Single timer, shared to implement all the timers managed by the Timer class.
// Not intended to be used directly; use the Timer class instead.
void setSharedTimerFiredFunction(void (*f)())
{
    if (JavaSharedClient::GetTimerClient())
        JavaSharedClient::GetTimerClient()->setSharedTimerCallback(f);
}

// The fire time is relative to the classic POSIX epoch of January 1, 1970,
// as the result of currentTime() is.
void setSharedTimerFireTime(double fireTime)
{
    long long timeInMs = static_cast<long long>((fireTime - WTF::currentTime()) * 1000);

    LOGV("setSharedTimerFireTime: in %ld millisec", timeInMs);
    if (JavaSharedClient::GetTimerClient())
        JavaSharedClient::GetTimerClient()->setSharedTimer(timeInMs);
}

void stopSharedTimer()
{
    if (JavaSharedClient::GetTimerClient())
        JavaSharedClient::GetTimerClient()->stopSharedTimer();
}

}  // namespace WebCore
