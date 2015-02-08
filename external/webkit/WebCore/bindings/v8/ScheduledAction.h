

#ifndef ScheduledAction_h
#define ScheduledAction_h

#include "OwnHandle.h"
#include "ScriptSourceCode.h"
#include "V8GCController.h"

#include <v8.h>

namespace WebCore {

    class String;
    class ScriptExecutionContext;
    class V8Proxy;
    class WorkerContext;

    class ScheduledAction {
    public:
        ScheduledAction(v8::Handle<v8::Context>, v8::Handle<v8::Function>, int argc, v8::Handle<v8::Value> argv[]);
        explicit ScheduledAction(v8::Handle<v8::Context> context, const WebCore::String& code, const KURL& url = KURL())
            : m_context(context)
            , m_argc(0)
            , m_argv(0)
            , m_code(code, url)
        {
        }

        virtual ~ScheduledAction();
        virtual void execute(ScriptExecutionContext*);

    private:
        void execute(V8Proxy*);
#if ENABLE(WORKERS)
        void execute(WorkerContext*);
#endif

        OwnHandle<v8::Context> m_context;
        v8::Persistent<v8::Function> m_function;
        int m_argc;
        v8::Persistent<v8::Value>* m_argv;
        ScriptSourceCode m_code;
    };

} // namespace WebCore

#endif // ScheduledAction
