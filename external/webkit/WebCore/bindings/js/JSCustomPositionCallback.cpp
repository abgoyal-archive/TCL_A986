

#include "config.h"
#include "JSCustomPositionCallback.h"

#include "Frame.h"
#include "JSGeoposition.h"
#include "ScriptController.h"
#include <runtime/JSLock.h>

namespace WebCore {

using namespace JSC;

JSCustomPositionCallback::JSCustomPositionCallback(JSObject* callback, JSDOMGlobalObject* globalObject)
    : m_data(callback, globalObject)
{
}

void JSCustomPositionCallback::handleEvent(Geoposition* geoposition)
{
    RefPtr<JSCustomPositionCallback> protect(this);

    JSC::JSLock lock(SilenceAssertionsOnly);
    ExecState* exec = m_data.globalObject()->globalExec();
    MarkedArgumentBuffer args;
    args.append(toJS(exec, deprecatedGlobalObjectForPrototype(exec), geoposition));
    m_data.invokeCallback(args);
}

} // namespace WebCore
