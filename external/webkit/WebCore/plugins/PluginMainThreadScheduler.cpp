

#include "config.h"
#include "PluginMainThreadScheduler.h"
#include <wtf/StdLibExtras.h>

namespace WebCore {

PluginMainThreadScheduler& PluginMainThreadScheduler::scheduler()
{
    DEFINE_STATIC_LOCAL(PluginMainThreadScheduler, scheduler, ());

    return scheduler;
}

PluginMainThreadScheduler::PluginMainThreadScheduler()
    : m_callPending(false)
{
}

void PluginMainThreadScheduler::scheduleCall(NPP npp, MainThreadFunction function, void* userData)
{
    MutexLocker lock(m_queueMutex);

    CallQueueMap::iterator it = m_callQueueMap.find(npp);
    if (it == m_callQueueMap.end())
        return;

    it->second.append(Call(function, userData));

    if (!m_callPending) {
        callOnMainThread(mainThreadCallback, this);
        m_callPending = true;
    }
}

void PluginMainThreadScheduler::registerPlugin(NPP npp)
{
    MutexLocker lock(m_queueMutex);

    ASSERT(!m_callQueueMap.contains(npp));
    m_callQueueMap.set(npp, Deque<Call>());
}

void PluginMainThreadScheduler::unregisterPlugin(NPP npp)
{
    MutexLocker lock(m_queueMutex);

    ASSERT(m_callQueueMap.contains(npp));
    m_callQueueMap.remove(npp);
}

void PluginMainThreadScheduler::dispatchCallsForPlugin(NPP npp, const Deque<Call>& calls)
{
    Deque<Call>::const_iterator end = calls.end();
    for (Deque<Call>::const_iterator it = calls.begin(); it != end; ++it) {
        // Check if the plug-in has been destroyed.
        {
            MutexLocker lock(m_queueMutex);
            if (!m_callQueueMap.contains(npp))
                return;
        }

        (*it).performCall();
    }
}

void PluginMainThreadScheduler::dispatchCalls()
{
    m_queueMutex.lock();
    CallQueueMap copy(m_callQueueMap);

    {
        // Empty all the queues in the original map
        CallQueueMap::iterator end = m_callQueueMap.end();
        for (CallQueueMap::iterator it = m_callQueueMap.begin(); it != end; ++it)
            it->second.clear();
    }

    m_callPending = false;
    m_queueMutex.unlock();

    CallQueueMap::iterator end = copy.end();
    for (CallQueueMap::iterator it = copy.begin(); it != end; ++it)
        dispatchCallsForPlugin(it->first, it->second);
}

void PluginMainThreadScheduler::mainThreadCallback(void* context)
{
    static_cast<PluginMainThreadScheduler*>(context)->dispatchCalls();
}

} // namespace WebCore
