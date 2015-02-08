

#ifndef WorkerScriptController_h
#define WorkerScriptController_h

#if ENABLE(WORKERS)

#include <runtime/Protect.h>
#include <wtf/Noncopyable.h>
#include <wtf/Threading.h>

namespace JSC {
    class JSGlobalData;
}

namespace WebCore {

    class JSWorkerContext;
    class ScriptSourceCode;
    class ScriptValue;
    class String;
    class WorkerContext;

    class WorkerScriptController : public Noncopyable {
    public:
        WorkerScriptController(WorkerContext*);
        ~WorkerScriptController();

        JSWorkerContext* workerContextWrapper()
        {
            initScriptIfNeeded();
            return m_workerContextWrapper;
        }

        ScriptValue evaluate(const ScriptSourceCode&);
        ScriptValue evaluate(const ScriptSourceCode&, ScriptValue* exception);

        void setException(ScriptValue);

        void forbidExecution();

        JSC::JSGlobalData* globalData() { return m_globalData.get(); }

    private:
        void initScriptIfNeeded()
        {
            if (!m_workerContextWrapper)
                initScript();
        }
        void initScript();

        RefPtr<JSC::JSGlobalData> m_globalData;
        WorkerContext* m_workerContext;
        JSC::ProtectedPtr<JSWorkerContext> m_workerContextWrapper;

        Mutex m_sharedDataMutex;
        bool m_executionForbidden;
    };

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // WorkerScriptController_h
