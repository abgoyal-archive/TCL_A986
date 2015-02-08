

#include "config.h"
#include "MainThread.h"

#include <glib.h>

namespace WTF {

void initializeMainThreadPlatform()
{
}

static gboolean timeoutFired(gpointer)
{
    dispatchFunctionsFromMainThread();
    return FALSE;
}

void scheduleDispatchFunctionsOnMainThread()
{
    g_timeout_add(0, timeoutFired, 0);
}

} // namespace WTF
