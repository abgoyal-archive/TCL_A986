

#include "config.h"
#include "Threading.h"

#if ENABLE(SINGLE_THREADED)

namespace WTF {

void initializeThreading() { }
ThreadIdentifier createThreadInternal(ThreadFunction, void*, const char*) { return ThreadIdentifier(); }
void initializeCurrentThreadInternal(const char*) { }
int waitForThreadCompletion(ThreadIdentifier, void**) { return 0; }
void detachThread(ThreadIdentifier) { }
ThreadIdentifier currentThread() { return ThreadIdentifier(); }
bool isMainThread() { return true; }

Mutex::Mutex() { }
Mutex::~Mutex() { }
void Mutex::lock() { }
bool Mutex::tryLock() { return false; }
void Mutex::unlock() { }

ThreadCondition::ThreadCondition() { }
ThreadCondition::~ThreadCondition() { }
void ThreadCondition::wait(Mutex&) { }
bool ThreadCondition::timedWait(Mutex&, double) { return false; }
void ThreadCondition::signal() { }
void ThreadCondition::broadcast() { }

void lockAtomicallyInitializedStaticMutex() { }
void unlockAtomicallyInitializedStaticMutex() { }

} // namespace WebCore

#endif
