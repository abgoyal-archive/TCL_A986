

#include "config.h"

#if ENABLE(WORKERS)

#include "WorkerScriptController.h"

#include "JSDOMBinding.h"
#include "JSDedicatedWorkerContext.h"
#include "JSSharedWorkerContext.h"
#include "ScriptSourceCode.h"
#include "ScriptValue.h"
#include "WorkerContext.h"
#include "WorkerObjectProxy.h"
#include "WorkerThread.h"
#include <interpreter/Interpreter.h>
#include <runtime/Completion.h>
#include <runtime/Completion.h>
#include <runtime/JSLock.h>

using namespace JSC;

namespace WebCore {

WorkerScriptController::WorkerScriptController(WorkerContext* workerContext)
    : m_globalData(JSGlobalData::create())
    , m_workerContext(workerContext)
    , m_executionForbidden(false)
{
    m_globalData->clientData = new WebCoreJSClientData(m_globalData.get());
}

WorkerScriptController::~WorkerScriptController()
{
    m_workerContextWrapper = 0; // Unprotect the global object.
    m_globalData->heap.destroy();
}

void WorkerScriptController::initScript()
{
    ASSERT(!m_workerContextWrapper);

    JSLock lock(SilenceAssertionsOnly);

    // Explicitly protect the global object's prototype so it isn't collected
    // when we allocate the global object. (Once the global object is fully
    // constructed, it can mark its own prototype.)
    RefPtr<Structure> workerContextPrototypeStructure = JSWorkerContextPrototype::createStructure(jsNull());
    ProtectedPtr<JSWorkerContextPrototype> workerContextPrototype = new (m_globalData.get()) JSWorkerContextPrototype(workerContextPrototypeStructure.release());

    if (m_workerContext->isDedicatedWorkerContext()) {
        RefPtr<Structure> dedicatedContextPrototypeStructure = JSDedicatedWorkerContextPrototype::createStructure(workerContextPrototype);
        ProtectedPtr<JSDedicatedWorkerContextPrototype> dedicatedContextPrototype = new (m_globalData.get()) JSDedicatedWorkerContextPrototype(dedicatedContextPrototypeStructure.release());
        RefPtr<Structure> structure = JSDedicatedWorkerContext::createStructure(dedicatedContextPrototype);

        m_workerContextWrapper = new (m_globalData.get()) JSDedicatedWorkerContext(structure.release(), m_workerContext->toDedicatedWorkerContext());
#if ENABLE(SHARED_WORKERS)
    } else {
        ASSERT(m_workerContext->isSharedWorkerContext());
        RefPtr<Structure> sharedContextPrototypeStructure = JSSharedWorkerContextPrototype::createStructure(workerContextPrototype);
        ProtectedPtr<JSSharedWorkerContextPrototype> sharedContextPrototype = new (m_globalData.get()) JSSharedWorkerContextPrototype(sharedContextPrototypeStructure.release());
        RefPtr<Structure> structure = JSSharedWorkerContext::createStructure(sharedContextPrototype);

        m_workerContextWrapper = new (m_globalData.get()) JSSharedWorkerContext(structure.release(), m_workerContext->toSharedWorkerContext());
#endif
    }
}

ScriptValue WorkerScriptController::evaluate(const ScriptSourceCode& sourceCode)
{
    {
        MutexLocker lock(m_sharedDataMutex);
        if (m_executionForbidden)
            return JSValue();
    }
    ScriptValue exception;
    ScriptValue result = evaluate(sourceCode, &exception);
    if (exception.jsValue()) {
        JSLock lock(SilenceAssertionsOnly);
        reportException(m_workerContextWrapper->globalExec(), exception.jsValue());
    }
    return result;
}

ScriptValue WorkerScriptController::evaluate(const ScriptSourceCode& sourceCode, ScriptValue* exception)
{
    {
        MutexLocker lock(m_sharedDataMutex);
        if (m_executionForbidden)
            return JSValue();
    }

    initScriptIfNeeded();
    JSLock lock(SilenceAssertionsOnly);

    ExecState* exec = m_workerContextWrapper->globalExec();
    m_workerContextWrapper->globalData()->timeoutChecker.start();
    Completion comp = JSC::evaluate(exec, exec->dynamicGlobalObject()->globalScopeChain(), sourceCode.jsSourceCode(), m_workerContextWrapper);
    m_workerContextWrapper->globalData()->timeoutChecker.stop();

    if (comp.complType() == Normal || comp.complType() == ReturnValue)
        return comp.value();

    if (comp.complType() == Throw)
        *exception = comp.value();
    return JSValue();
}

void WorkerScriptController::setException(ScriptValue exception)
{
    m_workerContextWrapper->globalExec()->setException(exception.jsValue());
}

void WorkerScriptController::forbidExecution()
{
    // This function is called from another thread.
    // Mutex protection for m_executionForbidden is needed to guarantee that the value is synchronized between processors, because
    // if it were not, the worker could re-enter JSC::evaluate(), but with timeout already reset.
    // It is not critical for Interpreter::m_timeoutTime to be synchronized, we just rely on it reaching the worker thread's processor sooner or later.
    MutexLocker lock(m_sharedDataMutex);
    m_executionForbidden = true;
    m_globalData->timeoutChecker.setTimeoutInterval(1); // 1ms is the smallest timeout that can be set.
}

} // namespace WebCore

#endif // ENABLE(WORKERS)
