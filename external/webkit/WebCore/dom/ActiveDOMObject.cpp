

#include "config.h"
#include "ActiveDOMObject.h"

#include "ScriptExecutionContext.h"
#include "WorkerContext.h"
#include "WorkerThread.h"

namespace WebCore {

ActiveDOMObject::ActiveDOMObject(ScriptExecutionContext* scriptExecutionContext, void* upcastPointer)
    : m_scriptExecutionContext(scriptExecutionContext)
    , m_pendingActivityCount(0)
{
#if ENABLE(WORKERS)
    ASSERT((m_scriptExecutionContext->isDocument() && isMainThread())
        || (m_scriptExecutionContext->isWorkerContext() && currentThread() == static_cast<WorkerContext*>(m_scriptExecutionContext)->thread()->threadID()));
#endif

    m_scriptExecutionContext->createdActiveDOMObject(this, upcastPointer);
}

ActiveDOMObject::~ActiveDOMObject()
{
    if (m_scriptExecutionContext) {
#if ENABLE(WORKERS)
        ASSERT((m_scriptExecutionContext->isDocument() && isMainThread())
            || (m_scriptExecutionContext->isWorkerContext() && currentThread() == static_cast<WorkerContext*>(m_scriptExecutionContext)->thread()->threadID()));
#endif

        m_scriptExecutionContext->destroyedActiveDOMObject(this);
    }
}

bool ActiveDOMObject::hasPendingActivity() const
{
    return m_pendingActivityCount;
}

void ActiveDOMObject::contextDestroyed()
{
    m_scriptExecutionContext = 0;
}

bool ActiveDOMObject::canSuspend() const
{
    return false;
}

void ActiveDOMObject::suspend()
{
}

void ActiveDOMObject::resume()
{
}

void ActiveDOMObject::stop()
{
}

} // namespace WebCore
