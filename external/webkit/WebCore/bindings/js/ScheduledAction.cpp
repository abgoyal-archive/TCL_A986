

#include "config.h"
#include "ScheduledAction.h"

#include "CString.h"
#include "DOMWindow.h"
#include "Document.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "JSDOMBinding.h"
#include "JSDOMWindow.h"
#include "ScriptController.h"
#include "ScriptExecutionContext.h"
#include "ScriptSourceCode.h"
#include "ScriptValue.h"
#include <runtime/JSLock.h>

#if ENABLE(WORKERS)
#include "JSWorkerContext.h"
#include "WorkerContext.h"
#include "WorkerThread.h"
#endif

using namespace JSC;

namespace WebCore {

ScheduledAction* ScheduledAction::create(ExecState* exec, const ArgList& args, DOMWrapperWorld* isolatedWorld)
{
    JSValue v = args.at(0);
    CallData callData;
    if (v.getCallData(callData) == CallTypeNone) {
        UString string = v.toString(exec);
        if (exec->hadException())
            return 0;
        return new ScheduledAction(string, isolatedWorld);
    }
    ArgList argsTail;
    args.getSlice(2, argsTail);
    return new ScheduledAction(v, argsTail, isolatedWorld);
}

ScheduledAction::ScheduledAction(JSValue function, const ArgList& args, DOMWrapperWorld* isolatedWorld)
    : m_function(function)
    , m_isolatedWorld(isolatedWorld)
{
    ArgList::const_iterator end = args.end();
    for (ArgList::const_iterator it = args.begin(); it != end; ++it)
        m_args.append(*it);
}

void ScheduledAction::execute(ScriptExecutionContext* context)
{
    if (context->isDocument())
        execute(static_cast<Document*>(context));
#if ENABLE(WORKERS)
    else {
        ASSERT(context->isWorkerContext());
        execute(static_cast<WorkerContext*>(context));
    }
#else
    ASSERT(context->isDocument());
#endif
}

void ScheduledAction::executeFunctionInContext(JSGlobalObject* globalObject, JSValue thisValue)
{
    ASSERT(m_function);
    JSLock lock(SilenceAssertionsOnly);

    CallData callData;
    CallType callType = m_function.get().getCallData(callData);
    if (callType == CallTypeNone)
        return;

    ExecState* exec = globalObject->globalExec();

    MarkedArgumentBuffer args;
    size_t size = m_args.size();
    for (size_t i = 0; i < size; ++i)
        args.append(m_args[i]);

    globalObject->globalData()->timeoutChecker.start();
    JSC::call(exec, m_function, callType, callData, thisValue, args);
    globalObject->globalData()->timeoutChecker.stop();

    if (exec->hadException())
        reportCurrentException(exec);
}

void ScheduledAction::execute(Document* document)
{
    JSDOMWindow* window = toJSDOMWindow(document->frame(), m_isolatedWorld.get());
    if (!window)
        return;

    RefPtr<Frame> frame = window->impl()->frame();
    if (!frame || !frame->script()->canExecuteScripts())
        return;

    frame->script()->setProcessingTimerCallback(true);

    if (m_function) {
        executeFunctionInContext(window, window->shell());
        Document::updateStyleForAllDocuments();
    } else
        frame->script()->executeScriptInWorld(m_isolatedWorld.get(), m_code);

    frame->script()->setProcessingTimerCallback(false);
}

#if ENABLE(WORKERS)
void ScheduledAction::execute(WorkerContext* workerContext)
{
    // In a Worker, the execution should always happen on a worker thread.
    ASSERT(workerContext->thread()->threadID() == currentThread());

    WorkerScriptController* scriptController = workerContext->script();

    if (m_function) {
        JSWorkerContext* contextWrapper = scriptController->workerContextWrapper();
        executeFunctionInContext(contextWrapper, contextWrapper);
    } else {
        ScriptSourceCode code(m_code, workerContext->url());
        scriptController->evaluate(code);
    }
}
#endif // ENABLE(WORKERS)

} // namespace WebCore
