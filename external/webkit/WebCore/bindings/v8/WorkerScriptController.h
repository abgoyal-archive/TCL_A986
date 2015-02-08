

#ifndef WorkerScriptController_h
#define WorkerScriptController_h

#if ENABLE(WORKERS)

#include <wtf/OwnPtr.h>
#include <wtf/Threading.h>

namespace WebCore {

    class ScriptSourceCode;
    class ScriptValue;
    class WorkerContext;
    class WorkerContextExecutionProxy;

    class WorkerScriptController {
    public:
        WorkerScriptController(WorkerContext*);
        ~WorkerScriptController();

        WorkerContextExecutionProxy* proxy() { return m_proxy.get(); }

        ScriptValue evaluate(const ScriptSourceCode&);
        ScriptValue evaluate(const ScriptSourceCode&, ScriptValue* exception);

        void setException(ScriptValue);

        void forbidExecution();

    private:
        WorkerContext* m_workerContext;
        OwnPtr<WorkerContextExecutionProxy> m_proxy;

        Mutex m_sharedDataMutex;
        bool m_executionForbidden;
    };

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // WorkerScriptController_h
