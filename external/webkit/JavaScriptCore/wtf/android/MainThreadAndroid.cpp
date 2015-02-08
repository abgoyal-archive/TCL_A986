

#include "config.h"
#include "MainThread.h"

#include "AndroidThreading.h"

namespace WTF {

void initializeMainThreadPlatform()
{
}

void scheduleDispatchFunctionsOnMainThread()
{
    AndroidThreading::scheduleDispatchFunctionsOnMainThread();
}

} // namespace WTF
