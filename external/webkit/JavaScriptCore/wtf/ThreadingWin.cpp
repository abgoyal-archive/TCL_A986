



#include "config.h"
#include "Threading.h"

#include "MainThread.h"
#if !USE(PTHREADS) && OS(WINDOWS)
#include "ThreadSpecific.h"
#endif
#if !OS(WINCE)
#include <process.h>
#endif
#if HAVE(ERRNO_H)
#include <errno.h>
#else
#define NO_ERRNO
#endif
#include <windows.h>
#include <wtf/CurrentTime.h>
#include <wtf/HashMap.h>
#include <wtf/MathExtras.h>
#include <wtf/RandomNumberSeed.h>

namespace WTF {

// MS_VC_EXCEPTION, THREADNAME_INFO, and setThreadNameInternal all come from <http://msdn.microsoft.com/en-us/library/xcb2z8hs.aspx>.
static const DWORD MS_VC_EXCEPTION = 0x406D1388;

#pragma pack(push, 8)
typedef struct tagTHREADNAME_INFO {
    DWORD dwType; // must be 0x1000
    LPCSTR szName; // pointer to name (in user addr space)
    DWORD dwThreadID; // thread ID (-1=caller thread)
    DWORD dwFlags; // reserved for future use, must be zero
} THREADNAME_INFO;
#pragma pack(pop)

void initializeCurrentThreadInternal(const char* szThreadName)
{
    THREADNAME_INFO info;
    info.dwType = 0x1000;
    info.szName = szThreadName;
    info.dwThreadID = GetCurrentThreadId();
    info.dwFlags = 0;

    __try {
        RaiseException(MS_VC_EXCEPTION, 0, sizeof(info)/sizeof(ULONG_PTR), reinterpret_cast<ULONG_PTR*>(&info));
    } __except (EXCEPTION_CONTINUE_EXECUTION) {
    }
}

static Mutex* atomicallyInitializedStaticMutex;

void lockAtomicallyInitializedStaticMutex()
{
    ASSERT(atomicallyInitializedStaticMutex);
    atomicallyInitializedStaticMutex->lock();
}

void unlockAtomicallyInitializedStaticMutex()
{
    atomicallyInitializedStaticMutex->unlock();
}

static ThreadIdentifier mainThreadIdentifier;

static Mutex& threadMapMutex()
{
    static Mutex mutex;
    return mutex;
}

void initializeThreading()
{
    if (!atomicallyInitializedStaticMutex) {
        atomicallyInitializedStaticMutex = new Mutex;
        threadMapMutex();
        initializeRandomNumberGenerator();
        initializeMainThread();
        mainThreadIdentifier = currentThread();
        initializeCurrentThreadInternal("Main Thread");
    }
}

static HashMap<DWORD, HANDLE>& threadMap()
{
    static HashMap<DWORD, HANDLE> map;
    return map;
}

static void storeThreadHandleByIdentifier(DWORD threadID, HANDLE threadHandle)
{
    MutexLocker locker(threadMapMutex());
    ASSERT(!threadMap().contains(threadID));
    threadMap().add(threadID, threadHandle);
}

static HANDLE threadHandleForIdentifier(ThreadIdentifier id)
{
    MutexLocker locker(threadMapMutex());
    return threadMap().get(id);
}

static void clearThreadHandleForIdentifier(ThreadIdentifier id)
{
    MutexLocker locker(threadMapMutex());
    ASSERT(threadMap().contains(id));
    threadMap().remove(id);
}

struct ThreadFunctionInvocation {
    ThreadFunctionInvocation(ThreadFunction function, void* data) : function(function), data(data) {}

    ThreadFunction function;
    void* data;
};

static unsigned __stdcall wtfThreadEntryPoint(void* param)
{
    ThreadFunctionInvocation invocation = *static_cast<ThreadFunctionInvocation*>(param);
    delete static_cast<ThreadFunctionInvocation*>(param);

    void* result = invocation.function(invocation.data);

#if !USE(PTHREADS) && OS(WINDOWS)
    // Do the TLS cleanup.
    ThreadSpecificThreadExit();
#endif

    return reinterpret_cast<unsigned>(result);
}

ThreadIdentifier createThreadInternal(ThreadFunction entryPoint, void* data, const char* threadName)
{
    unsigned threadIdentifier = 0;
    ThreadIdentifier threadID = 0;
    ThreadFunctionInvocation* invocation = new ThreadFunctionInvocation(entryPoint, data);
#if OS(WINCE)
    // This is safe on WINCE, since CRT is in the core and innately multithreaded.
    // On desktop Windows, need to use _beginthreadex (not available on WinCE) if using any CRT functions
    HANDLE threadHandle = CreateThread(0, 0, (LPTHREAD_START_ROUTINE)wtfThreadEntryPoint, invocation, 0, (LPDWORD)&threadIdentifier);
#else
    HANDLE threadHandle = reinterpret_cast<HANDLE>(_beginthreadex(0, 0, wtfThreadEntryPoint, invocation, 0, &threadIdentifier));
#endif
    if (!threadHandle) {
#if OS(WINCE)
        LOG_ERROR("Failed to create thread at entry point %p with data %p: %ld", entryPoint, data, ::GetLastError());
#elif defined(NO_ERRNO)
        LOG_ERROR("Failed to create thread at entry point %p with data %p.", entryPoint, data);
#else
        LOG_ERROR("Failed to create thread at entry point %p with data %p: %ld", entryPoint, data, errno);
#endif
        return 0;
    }

    threadID = static_cast<ThreadIdentifier>(threadIdentifier);
    storeThreadHandleByIdentifier(threadIdentifier, threadHandle);

    return threadID;
}

int waitForThreadCompletion(ThreadIdentifier threadID, void** result)
{
    ASSERT(threadID);
    
    HANDLE threadHandle = threadHandleForIdentifier(threadID);
    if (!threadHandle)
        LOG_ERROR("ThreadIdentifier %u did not correspond to an active thread when trying to quit", threadID);
 
    DWORD joinResult = WaitForSingleObject(threadHandle, INFINITE);
    if (joinResult == WAIT_FAILED)
        LOG_ERROR("ThreadIdentifier %u was found to be deadlocked trying to quit", threadID);

    CloseHandle(threadHandle);
    clearThreadHandleForIdentifier(threadID);

    return joinResult;
}

void detachThread(ThreadIdentifier threadID)
{
    ASSERT(threadID);

    HANDLE threadHandle = threadHandleForIdentifier(threadID);
    if (threadHandle)
        CloseHandle(threadHandle);
    clearThreadHandleForIdentifier(threadID);
}

ThreadIdentifier currentThread()
{
    return static_cast<ThreadIdentifier>(GetCurrentThreadId());
}

bool isMainThread()
{
    return currentThread() == mainThreadIdentifier;
}

Mutex::Mutex()
{
    m_mutex.m_recursionCount = 0;
    InitializeCriticalSection(&m_mutex.m_internalMutex);
}

Mutex::~Mutex()
{
    DeleteCriticalSection(&m_mutex.m_internalMutex);
}

void Mutex::lock()
{
    EnterCriticalSection(&m_mutex.m_internalMutex);
    ++m_mutex.m_recursionCount;
}
    
bool Mutex::tryLock()
{
    // This method is modeled after the behavior of pthread_mutex_trylock,
    // which will return an error if the lock is already owned by the
    // current thread.  Since the primitive Win32 'TryEnterCriticalSection'
    // treats this as a successful case, it changes the behavior of several
    // tests in WebKit that check to see if the current thread already
    // owned this mutex (see e.g., IconDatabase::getOrCreateIconRecord)
    DWORD result = TryEnterCriticalSection(&m_mutex.m_internalMutex);
    
    if (result != 0) {       // We got the lock
        // If this thread already had the lock, we must unlock and
        // return false so that we mimic the behavior of POSIX's
        // pthread_mutex_trylock:
        if (m_mutex.m_recursionCount > 0) {
            LeaveCriticalSection(&m_mutex.m_internalMutex);
            return false;
        }

        ++m_mutex.m_recursionCount;
        return true;
    }

    return false;
}

void Mutex::unlock()
{
    --m_mutex.m_recursionCount;
    LeaveCriticalSection(&m_mutex.m_internalMutex);
}

bool PlatformCondition::timedWait(PlatformMutex& mutex, DWORD durationMilliseconds)
{
    // Enter the wait state.
    DWORD res = WaitForSingleObject(m_blockLock, INFINITE);
    ASSERT(res == WAIT_OBJECT_0);
    ++m_waitersBlocked;
    res = ReleaseSemaphore(m_blockLock, 1, 0);
    ASSERT(res);

    LeaveCriticalSection(&mutex.m_internalMutex);

    // Main wait - use timeout.
    bool timedOut = (WaitForSingleObject(m_blockQueue, durationMilliseconds) == WAIT_TIMEOUT);

    res = WaitForSingleObject(m_unblockLock, INFINITE);
    ASSERT(res == WAIT_OBJECT_0);

    int signalsLeft = m_waitersToUnblock;

    if (m_waitersToUnblock)
        --m_waitersToUnblock;
    else if (++m_waitersGone == (INT_MAX / 2)) { // timeout/canceled or spurious semaphore
        // timeout or spurious wakeup occured, normalize the m_waitersGone count
        // this may occur if many calls to wait with a timeout are made and
        // no call to notify_* is made
        res = WaitForSingleObject(m_blockLock, INFINITE);
        ASSERT(res == WAIT_OBJECT_0);
        m_waitersBlocked -= m_waitersGone;
        res = ReleaseSemaphore(m_blockLock, 1, 0);
        ASSERT(res);
        m_waitersGone = 0;
    }

    res = ReleaseMutex(m_unblockLock);
    ASSERT(res);

    if (signalsLeft == 1) {
        res = ReleaseSemaphore(m_blockLock, 1, 0); // Open the gate.
        ASSERT(res);
    }

    EnterCriticalSection (&mutex.m_internalMutex);

    return !timedOut;
}

void PlatformCondition::signal(bool unblockAll)
{
    unsigned signalsToIssue = 0;

    DWORD res = WaitForSingleObject(m_unblockLock, INFINITE);
    ASSERT(res == WAIT_OBJECT_0);

    if (m_waitersToUnblock) { // the gate is already closed
        if (!m_waitersBlocked) { // no-op
            res = ReleaseMutex(m_unblockLock);
            ASSERT(res);
            return;
        }

        if (unblockAll) {
            signalsToIssue = m_waitersBlocked;
            m_waitersToUnblock += m_waitersBlocked;
            m_waitersBlocked = 0;
        } else {
            signalsToIssue = 1;
            ++m_waitersToUnblock;
            --m_waitersBlocked;
        }
    } else if (m_waitersBlocked > m_waitersGone) {
        res = WaitForSingleObject(m_blockLock, INFINITE); // Close the gate.
        ASSERT(res == WAIT_OBJECT_0);
        if (m_waitersGone != 0) {
            m_waitersBlocked -= m_waitersGone;
            m_waitersGone = 0;
        }
        if (unblockAll) {
            signalsToIssue = m_waitersBlocked;
            m_waitersToUnblock = m_waitersBlocked;
            m_waitersBlocked = 0;
        } else {
            signalsToIssue = 1;
            m_waitersToUnblock = 1;
            --m_waitersBlocked;
        }
    } else { // No-op.
        res = ReleaseMutex(m_unblockLock);
        ASSERT(res);
        return;
    }

    res = ReleaseMutex(m_unblockLock);
    ASSERT(res);

    if (signalsToIssue) {
        res = ReleaseSemaphore(m_blockQueue, signalsToIssue, 0);
        ASSERT(res);
    }
}

static const long MaxSemaphoreCount = static_cast<long>(~0UL >> 1);

ThreadCondition::ThreadCondition()
{
    m_condition.m_waitersGone = 0;
    m_condition.m_waitersBlocked = 0;
    m_condition.m_waitersToUnblock = 0;
    m_condition.m_blockLock = CreateSemaphore(0, 1, 1, 0);
    m_condition.m_blockQueue = CreateSemaphore(0, 0, MaxSemaphoreCount, 0);
    m_condition.m_unblockLock = CreateMutex(0, 0, 0);

    if (!m_condition.m_blockLock || !m_condition.m_blockQueue || !m_condition.m_unblockLock) {
        if (m_condition.m_blockLock)
            CloseHandle(m_condition.m_blockLock);
        if (m_condition.m_blockQueue)
            CloseHandle(m_condition.m_blockQueue);
        if (m_condition.m_unblockLock)
            CloseHandle(m_condition.m_unblockLock);
    }
}

ThreadCondition::~ThreadCondition()
{
    CloseHandle(m_condition.m_blockLock);
    CloseHandle(m_condition.m_blockQueue);
    CloseHandle(m_condition.m_unblockLock);
}

void ThreadCondition::wait(Mutex& mutex)
{
    m_condition.timedWait(mutex.impl(), INFINITE);
}

bool ThreadCondition::timedWait(Mutex& mutex, double absoluteTime)
{
    double currentTime = WTF::currentTime();

    // Time is in the past - return immediately.
    if (absoluteTime < currentTime)
        return false;

    // Time is too far in the future (and would overflow unsigned long) - wait forever.
    if (absoluteTime - currentTime > static_cast<double>(INT_MAX) / 1000.0) {
        wait(mutex);
        return true;
    }

    double intervalMilliseconds = (absoluteTime - currentTime) * 1000.0;
    return m_condition.timedWait(mutex.impl(), static_cast<unsigned long>(intervalMilliseconds));
}

void ThreadCondition::signal()
{
    m_condition.signal(false); // Unblock only 1 thread.
}

void ThreadCondition::broadcast()
{
    m_condition.signal(true); // Unblock all threads.
}

} // namespace WTF
