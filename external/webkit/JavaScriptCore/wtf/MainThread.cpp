

#include "config.h"
#include "MainThread.h"

#include "StdLibExtras.h"
#include "CurrentTime.h"
#include "Deque.h"
#include "Threading.h"

namespace WTF {

struct FunctionWithContext {
    MainThreadFunction* function;
    void* context;
    ThreadCondition* syncFlag;

    FunctionWithContext(MainThreadFunction* function = 0, void* context = 0, ThreadCondition* syncFlag = 0)
        : function(function)
        , context(context)
        , syncFlag(syncFlag)
    { 
    }
};

typedef Deque<FunctionWithContext> FunctionQueue;

static bool callbacksPaused; // This global variable is only accessed from main thread.

Mutex& mainThreadFunctionQueueMutex()
{
    DEFINE_STATIC_LOCAL(Mutex, staticMutex, ());
    return staticMutex;
}

static FunctionQueue& functionQueue()
{
    DEFINE_STATIC_LOCAL(FunctionQueue, staticFunctionQueue, ());
    return staticFunctionQueue;
}

void initializeMainThread()
{
    mainThreadFunctionQueueMutex();
    initializeMainThreadPlatform();
}

// 0.1 sec delays in UI is approximate threshold when they become noticeable. Have a limit that's half of that.
static const double maxRunLoopSuspensionTime = 0.05;

void dispatchFunctionsFromMainThread()
{
    ASSERT(isMainThread());

    if (callbacksPaused)
        return;

    double startTime = currentTime();

    FunctionWithContext invocation;
    while (true) {
        {
            MutexLocker locker(mainThreadFunctionQueueMutex());
            if (!functionQueue().size())
                break;
            invocation = functionQueue().first();
            functionQueue().removeFirst();
        }

        invocation.function(invocation.context);
        if (invocation.syncFlag)
            invocation.syncFlag->signal();

        // If we are running accumulated functions for too long so UI may become unresponsive, we need to
        // yield so the user input can be processed. Otherwise user may not be able to even close the window.
        // This code has effect only in case the scheduleDispatchFunctionsOnMainThread() is implemented in a way that
        // allows input events to be processed before we are back here.
        if (currentTime() - startTime > maxRunLoopSuspensionTime) {
            scheduleDispatchFunctionsOnMainThread();
            break;
        }
    }
}

void callOnMainThread(MainThreadFunction* function, void* context)
{
    ASSERT(function);
    bool needToSchedule = false;
    {
        MutexLocker locker(mainThreadFunctionQueueMutex());
        needToSchedule = functionQueue().size() == 0;
        functionQueue().append(FunctionWithContext(function, context));
    }
    if (needToSchedule)
        scheduleDispatchFunctionsOnMainThread();
}

void callOnMainThreadAndWait(MainThreadFunction* function, void* context)
{
    ASSERT(function);

    if (isMainThread()) {
        function(context);
        return;
    }

    ThreadCondition syncFlag;
    Mutex& functionQueueMutex = mainThreadFunctionQueueMutex();
    MutexLocker locker(functionQueueMutex);
    functionQueue().append(FunctionWithContext(function, context, &syncFlag));
    if (functionQueue().size() == 1)
        scheduleDispatchFunctionsOnMainThread();
    syncFlag.wait(functionQueueMutex);
}

void setMainThreadCallbacksPaused(bool paused)
{
    ASSERT(isMainThread());

    if (callbacksPaused == paused)
        return;

    callbacksPaused = paused;

    if (!callbacksPaused)
        scheduleDispatchFunctionsOnMainThread();
}

} // namespace WTF
