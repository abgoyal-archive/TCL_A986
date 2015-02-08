

#include "config.h"
#include "SharedTimer.h"

#include <AEEAppGen.h>
#include <AEEStdLib.h>

#include <wtf/CurrentTime.h>

namespace WebCore {

static void (*sharedTimerFiredFunction)();

static AEECallback sharedTimerCallback;

static void invokeCallback(void*)
{
    sharedTimerFiredFunction();
}

void setSharedTimerFiredFunction(void (*f)())
{
    sharedTimerFiredFunction = f;
}

void setSharedTimerFireTime(double fireTime)
{
    ASSERT(sharedTimerFiredFunction);

    CALLBACK_Cancel(&sharedTimerCallback);

    double interval = fireTime - currentTime();
    int intervalInMS;

    if (interval < 0)
        intervalInMS = 0;
    else {
        interval *= 1000;
        intervalInMS = static_cast<int>(interval);
    }

    sharedTimerCallback.pfnCancel = 0;
    sharedTimerCallback.pfnNotify = invokeCallback;
    sharedTimerCallback.pNotifyData = 0;

    IShell* shell = reinterpret_cast<AEEApplet*>(GETAPPINSTANCE())->m_pIShell;
    ISHELL_SetTimerEx(shell, intervalInMS, &sharedTimerCallback);
}

void stopSharedTimer()
{
    CALLBACK_Cancel(&sharedTimerCallback);
}

}
