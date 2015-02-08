

#include "config.h"
#include "SharedTimer.h"

#include <wtf/Assertions.h>
#include <wtf/CurrentTime.h>
#include <glib.h>

namespace WebCore {

static guint sharedTimer;
static void (*sharedTimerFiredFunction)();

void setSharedTimerFiredFunction(void (*f)())
{
    sharedTimerFiredFunction = f;
}

static gboolean timeout_cb(gpointer)
{
    if (sharedTimerFiredFunction)
        sharedTimerFiredFunction();
    return FALSE;
}

void setSharedTimerFireTime(double fireTime)
{
    ASSERT(sharedTimerFiredFunction);

    double interval = fireTime - currentTime();
    guint intervalInMS;
    if (interval < 0)
        intervalInMS = 0;
    else {
        interval *= 1000;
        intervalInMS = (guint)interval;
    }

    stopSharedTimer();
    if (intervalInMS == 0)
        sharedTimer = g_idle_add(timeout_cb, NULL);
    else
        sharedTimer = g_timeout_add_full(G_PRIORITY_DEFAULT, intervalInMS, timeout_cb, NULL, NULL);
}

void stopSharedTimer()
{
    gboolean s = FALSE;
    if (sharedTimer == 0)
        return;

    s = g_source_remove(sharedTimer);
    ASSERT(s);
    sharedTimer = 0;
}

}
