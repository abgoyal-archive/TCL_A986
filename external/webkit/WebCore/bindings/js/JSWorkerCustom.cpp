

#include "config.h"

#if ENABLE(WORKERS)

#include "JSWorker.h"

#include "JSDOMGlobalObject.h"
#include "JSMessagePortCustom.h"
#include "Worker.h"

using namespace JSC;

namespace WebCore {

JSC::JSValue JSWorker::postMessage(JSC::ExecState* exec, const JSC::ArgList& args)
{
    return handlePostMessage(exec, args, impl());
}

} // namespace WebCore

#endif // ENABLE(WORKERS)
