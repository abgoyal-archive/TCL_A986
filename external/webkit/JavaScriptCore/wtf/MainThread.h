

#ifndef MainThread_h
#define MainThread_h

namespace WTF {

class Mutex;

typedef void MainThreadFunction(void*);

void callOnMainThread(MainThreadFunction*, void* context);

// Blocks the thread until the call finishes on the main thread. Misusing this can easily cause deadlocks.
void callOnMainThreadAndWait(MainThreadFunction*, void* context);

void setMainThreadCallbacksPaused(bool paused);

// Must be called from the main thread (Darwin is an exception to this rule).
void initializeMainThread();

// These functions are internal to the callOnMainThread implementation.
void initializeMainThreadPlatform();
void scheduleDispatchFunctionsOnMainThread();
Mutex& mainThreadFunctionQueueMutex();
void dispatchFunctionsFromMainThread();

} // namespace WTF

using WTF::callOnMainThread;
using WTF::callOnMainThreadAndWait;
using WTF::setMainThreadCallbacksPaused;

#endif // MainThread_h
