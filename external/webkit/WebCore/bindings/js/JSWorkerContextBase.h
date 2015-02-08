

#ifndef JSWorkerContextBase_h
#define JSWorkerContextBase_h

#if ENABLE(WORKERS)

#include "JSDOMGlobalObject.h"

namespace WebCore {

    class JSDedicatedWorkerContext;
    class JSSharedWorkerContext;
    class JSWorkerContext;
    class WorkerContext;

    class JSWorkerContextBase : public JSDOMGlobalObject {
        typedef JSDOMGlobalObject Base;
    public:
        JSWorkerContextBase(NonNullPassRefPtr<JSC::Structure>, PassRefPtr<WorkerContext>);
        virtual ~JSWorkerContextBase();

        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
        static const JSC::ClassInfo s_info;

        WorkerContext* impl() const { return m_impl.get(); }
        virtual ScriptExecutionContext* scriptExecutionContext() const;

    private:
        RefPtr<WorkerContext> m_impl;
    };

    // Returns a JSWorkerContext or jsNull()
    // Always ignores the execState and passed globalObject, WorkerContext is itself a globalObject and will always use its own prototype chain.
    JSC::JSValue toJS(JSC::ExecState*, JSDOMGlobalObject*, WorkerContext*);
    JSC::JSValue toJS(JSC::ExecState*, WorkerContext*);

    JSDedicatedWorkerContext* toJSDedicatedWorkerContext(JSC::JSValue);
    JSWorkerContext* toJSWorkerContext(JSC::JSValue);

#if ENABLE(SHARED_WORKERS)
    JSSharedWorkerContext* toJSSharedWorkerContext(JSC::JSValue);
#endif

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // JSWorkerContextBase_h
