

#include "config.h"
#include "GCController.h"

#include "JSDOMWindow.h"
#include <runtime/JSGlobalData.h>
#include <runtime/JSLock.h>
#include <runtime/Collector.h>
#include <wtf/StdLibExtras.h>

#if USE(PTHREADS)
#include <pthread.h>
#endif

using namespace JSC;

namespace WebCore {

static void* collect(void*)
{
    JSLock lock(SilenceAssertionsOnly);
    JSDOMWindow::commonJSGlobalData()->heap.collectAllGarbage();
    return 0;
}

GCController& gcController()
{
    DEFINE_STATIC_LOCAL(GCController, staticGCController, ());
    return staticGCController;
}

GCController::GCController()
    : m_GCTimer(this, &GCController::gcTimerFired)
{
}

void GCController::garbageCollectSoon()
{
    if (!m_GCTimer.isActive())
        m_GCTimer.startOneShot(0.5);
}

void GCController::gcTimerFired(Timer<GCController>*)
{
    collect(0);
}

void GCController::garbageCollectNow()
{
    collect(0);
}

void GCController::garbageCollectOnAlternateThreadForDebugging(bool waitUntilDone)
{
#if USE(PTHREADS)
    pthread_t thread;
    pthread_create(&thread, NULL, collect, NULL);

    if (waitUntilDone)
        pthread_join(thread, NULL);
#endif
}

} // namespace WebCore
