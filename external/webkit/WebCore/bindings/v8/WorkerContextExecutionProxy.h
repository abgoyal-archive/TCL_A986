


#ifndef WorkerContextExecutionProxy_h
#define WorkerContextExecutionProxy_h

#if ENABLE(WORKERS)

#include "ScriptValue.h"
#include "V8EventListenerList.h"
#include "V8Index.h"
#include <v8.h>
#include <wtf/OwnPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

    class Event;
    class EventTarget;
    class V8EventListener;
    class V8WorkerContextEventListener;
    class WorkerContext;

    struct WorkerContextExecutionState {
        WorkerContextExecutionState() : hadException(false), lineNumber(0) { }

        bool hadException;
        ScriptValue exception;
        String errorMessage;
        int lineNumber;
        String sourceURL;
    };

    class WorkerContextExecutionProxy {
    public:
        WorkerContextExecutionProxy(WorkerContext*);
        ~WorkerContextExecutionProxy();

        // Finds/creates event listener wrappers.
        PassRefPtr<V8EventListener> findOrCreateEventListener(v8::Local<v8::Value> listener, bool isInline, bool findOnly);

        // Track the event so that we can detach it from the JS wrapper when a worker
        // terminates. This is needed because we need to be able to dispose these
        // events and releases references to their event targets: WorkerContext.
        void trackEvent(Event*);

        // Evaluate a script file in the current execution environment.
        ScriptValue evaluate(const String& script, const String& fileName, int baseLine, WorkerContextExecutionState*);

        // Returns a local handle of the context.
        v8::Local<v8::Context> context() { return v8::Local<v8::Context>::New(m_context); }

        // Returns WorkerContext object.
        WorkerContext* workerContext() { return m_workerContext; }

        // Returns WorkerContextExecutionProxy object of the currently executing context. 0 will be returned if the current executing context is not the worker context.
        static WorkerContextExecutionProxy* retrieve();

    private:
        void initV8IfNeeded();
        void initContextIfNeeded();
        void dispose();

        // Run an already compiled script.
        v8::Local<v8::Value> runScript(v8::Handle<v8::Script>);

        static bool forgetV8EventObject(Event*);

        static const int kWorkerMaxStackSize = 500 * 1024;

        WorkerContext* m_workerContext;
        v8::Persistent<v8::Context> m_context;
        int m_recursion;

        Vector<Event*> m_events;
    };

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // WorkerContextExecutionProxy_h
