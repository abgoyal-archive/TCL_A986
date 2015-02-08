

#include "config.h"

#if USE(PTHREADS)

#include "ThreadIdentifierDataPthreads.h"

#include "Threading.h"

namespace WTF {

pthread_key_t ThreadIdentifierData::m_key;

void clearPthreadHandleForIdentifier(ThreadIdentifier);

ThreadIdentifierData::~ThreadIdentifierData()
{
    clearPthreadHandleForIdentifier(m_identifier);
}

ThreadIdentifier ThreadIdentifierData::identifier()
{
    initializeKeyOnce();
    ThreadIdentifierData* threadIdentifierData = static_cast<ThreadIdentifierData*>(pthread_getspecific(m_key));

    return threadIdentifierData ? threadIdentifierData->m_identifier : 0;
}

void ThreadIdentifierData::initialize(ThreadIdentifier id)
{
    ASSERT(!identifier());

    initializeKeyOnce();
    pthread_setspecific(m_key, new ThreadIdentifierData(id));
}

void ThreadIdentifierData::destruct(void* data)
{
    ThreadIdentifierData* threadIdentifierData = static_cast<ThreadIdentifierData*>(data);
    ASSERT(threadIdentifierData);

    if (threadIdentifierData->m_isDestroyedOnce) {
        delete threadIdentifierData;
        return;
    }

    threadIdentifierData->m_isDestroyedOnce = true;
    // Re-setting the value for key causes another destruct() call after all other thread-specific destructors were called.
    pthread_setspecific(m_key, threadIdentifierData);
}

void ThreadIdentifierData::initializeKeyOnceHelper()
{
    if (pthread_key_create(&m_key, destruct))
        CRASH();
}

void ThreadIdentifierData::initializeKeyOnce()
{
    static pthread_once_t onceControl = PTHREAD_ONCE_INIT;
    if (pthread_once(&onceControl, initializeKeyOnceHelper))
        CRASH();
}

} // namespace WTF

#endif // USE(PTHREADS)

