

#include "config.h"
#include "MainThread.h"

#include "ChromiumThreading.h"

namespace WTF {

void initializeMainThreadPlatform()
{
    ChromiumThreading::initializeMainThread();
}

void scheduleDispatchFunctionsOnMainThread()
{
    ChromiumThreading::scheduleDispatchFunctionsOnMainThread();
}

} // namespace WTF

