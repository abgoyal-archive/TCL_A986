

#include "config.h"
#include "JSCustomPositionErrorCallback.h"

#include "Frame.h"
#include "JSPositionError.h"
#include "ScriptController.h"
#include <runtime/JSLock.h>

namespace WebCore {
    
using namespace JSC;

JSCustomPositionErrorCallback::JSCustomPositionErrorCallback(JSObject* callback, JSDOMGlobalObject* globalObject)
    : m_data(callback, globalObject)
{
}

void JSCustomPositionErrorCallback::handleEvent(PositionError* positionError)
{
    RefPtr<JSCustomPositionErrorCallback> protect(this);

    JSC::JSLock lock(SilenceAssertionsOnly);
    ExecState* exec = m_data.globalObject()->globalExec();
    MarkedArgumentBuffer args;
    args.append(toJS(exec, deprecatedGlobalObjectForPrototype(exec), positionError));
    
    m_data.invokeCallback(args);
}
    
} // namespace WebCore
