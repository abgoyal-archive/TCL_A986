

#include "config.h"
#include "JSCustomApplicationInstalledCallback.h"

#if PLATFORM(ANDROID) && ENABLE(APPLICATION_INSTALLED)

#include "ScriptController.h"
#include <runtime/JSLock.h>

namespace WebCore {

using namespace JSC;

JSCustomApplicationInstalledCallback::JSCustomApplicationInstalledCallback(JSObject* callback, JSDOMGlobalObject* globalObject)
    : m_data(callback, globalObject)
{
}

void JSCustomApplicationInstalledCallback::handleEvent(bool isInstalled)
{
    RefPtr<JSCustomApplicationInstalledCallback> protect(this);

    JSC::JSLock lock(SilenceAssertionsOnly);
    ExecState* exec = m_data.globalObject()->globalExec();
    MarkedArgumentBuffer args;
    args.append(jsBoolean(isInstalled));
    m_data.invokeCallback(args);
}

} // namespace WebCore

#endif // PLATFORM(ANDROID) && ENABLE(APPLICATION_INSTALLED)
