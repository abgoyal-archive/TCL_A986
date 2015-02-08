

#include "config.h"

#if ENABLE(WORKERS)

#include "WorkerScriptController.h"

#include <v8.h>

#include "ScriptSourceCode.h"
#include "ScriptValue.h"
#include "DOMTimer.h"
#include "V8DOMMap.h"
#include "V8Proxy.h"
#include "WorkerContext.h"
#include "WorkerContextExecutionProxy.h"
#include "WorkerObjectProxy.h"
#include "WorkerThread.h"

namespace WebCore {

WorkerScriptController::WorkerScriptController(WorkerContext* workerContext)
    : m_workerContext(workerContext)
    , m_proxy(new WorkerContextExecutionProxy(workerContext))
    , m_executionForbidden(false)
{
}

WorkerScriptController::~WorkerScriptController()
{
    removeAllDOMObjectsInCurrentThread();
}

ScriptValue WorkerScriptController::evaluate(const ScriptSourceCode& sourceCode)
{
    return evaluate(sourceCode, 0);
}

ScriptValue WorkerScriptController::evaluate(const ScriptSourceCode& sourceCode, ScriptValue* exception)
{
    {
        MutexLocker lock(m_sharedDataMutex);
        if (m_executionForbidden)
            return ScriptValue();
    }

    WorkerContextExecutionState state;
    ScriptValue result = m_proxy->evaluate(sourceCode.source(), sourceCode.url().string(), sourceCode.startLine() - 1, &state);
    if (state.hadException) {
        if (exception)
            *exception = state.exception;
        else
            m_workerContext->reportException(state.errorMessage, state.lineNumber, state.sourceURL);
    }

    return result;
}

void WorkerScriptController::forbidExecution()
{
    // This function is called from another thread.
    MutexLocker lock(m_sharedDataMutex);
    m_executionForbidden = true;
}

void WorkerScriptController::setException(ScriptValue exception)
{
    throwError(*exception.v8Value());
}

} // namespace WebCore

#endif // ENABLE(WORKERS)
