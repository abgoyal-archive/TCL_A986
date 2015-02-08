

#include "config.h"

#if ENABLE(WORKERS)

#include "JSWorkerContextBase.h"

#include "JSDedicatedWorkerContext.h"
#include "JSSharedWorkerContext.h"
#include "JSWorkerContext.h"
#include "WorkerContext.h"

using namespace JSC;

namespace WebCore {

ASSERT_CLASS_FITS_IN_CELL(JSWorkerContextBase);

const ClassInfo JSWorkerContextBase::s_info = { "WorkerContext", &JSDOMGlobalObject::s_info, 0, 0 };

JSWorkerContextBase::JSWorkerContextBase(NonNullPassRefPtr<JSC::Structure> structure, PassRefPtr<WorkerContext> impl)
    : JSDOMGlobalObject(structure, new JSDOMGlobalObjectData(normalWorld(*impl->script()->globalData())), this)
    , m_impl(impl)
{
}

JSWorkerContextBase::~JSWorkerContextBase()
{
}

ScriptExecutionContext* JSWorkerContextBase::scriptExecutionContext() const
{
    return m_impl.get();
}

JSValue toJS(ExecState* exec, JSDOMGlobalObject*, WorkerContext* workerContext)
{
    return toJS(exec, workerContext);
}

JSValue toJS(ExecState*, WorkerContext* workerContext)
{
    if (!workerContext)
        return jsNull();
    WorkerScriptController* script = workerContext->script();
    if (!script)
        return jsNull();
    return script->workerContextWrapper();
}

JSDedicatedWorkerContext* toJSDedicatedWorkerContext(JSValue value)
{
    if (!value.isObject())
        return 0;
    const ClassInfo* classInfo = asObject(value)->classInfo();
    if (classInfo == &JSDedicatedWorkerContext::s_info)
        return static_cast<JSDedicatedWorkerContext*>(asObject(value));
    return 0;
}

#if ENABLE(SHARED_WORKERS)
JSSharedWorkerContext* toJSSharedWorkerContext(JSValue value)
{
    if (!value.isObject())
        return 0;
    const ClassInfo* classInfo = asObject(value)->classInfo();
    if (classInfo == &JSSharedWorkerContext::s_info)
        return static_cast<JSSharedWorkerContext*>(asObject(value));
    return 0;
}
#endif

JSWorkerContext* toJSWorkerContext(JSValue value)
{
    JSWorkerContext* context = toJSDedicatedWorkerContext(value);
#if ENABLE(SHARED_WORKERS)
    if (!context)
        context = toJSSharedWorkerContext(value);
#endif
    return context;
}

} // namespace WebCore

#endif // ENABLE(WORKERS)
