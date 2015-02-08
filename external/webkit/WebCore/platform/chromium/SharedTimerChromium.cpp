

#include "config.h"
#include "SharedTimer.h"

#include "ChromiumBridge.h"

namespace WebCore {

void setSharedTimerFiredFunction(void (*f)())
{                   
    ChromiumBridge::setSharedTimerFiredFunction(f);
}

void setSharedTimerFireTime(double fireTime)
{
    ChromiumBridge::setSharedTimerFireTime(fireTime);
}

void stopSharedTimer()
{
    ChromiumBridge::stopSharedTimer();
}

} // namespace WebCore
