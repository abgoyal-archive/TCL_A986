

#include "config.h"

#if ENABLE(WORKERS)

#include "JSDedicatedWorkerContext.h"

#include "JSDOMBinding.h"
#include "JSMessagePortCustom.h"

using namespace JSC;

namespace WebCore {

JSC::JSValue JSDedicatedWorkerContext::postMessage(JSC::ExecState* exec, const JSC::ArgList& args)
{
    return handlePostMessage(exec, args, impl());
}

} // namespace WebCore

#endif // ENABLE(WORKERS)
