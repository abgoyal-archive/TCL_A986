
#if PLATFORM(ANDROID) && ENABLE(APPLICATION_INSTALLED)

#ifndef JSCustomApplicationInstalledCallback_h
#define JSCustomApplicationInstalledCallback_h

#include "ApplicationInstalledCallback.h"
#include "JSCallbackData.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class JSCustomApplicationInstalledCallback : public ApplicationInstalledCallback {
public:
    static PassRefPtr<JSCustomApplicationInstalledCallback> create(JSC::JSObject* callback, JSDOMGlobalObject* globalObject)
    {
        return adoptRef(new JSCustomApplicationInstalledCallback(callback, globalObject));
    }

    virtual void handleEvent(bool isInstalled);

private:
    JSCustomApplicationInstalledCallback(JSC::JSObject* callback, JSDOMGlobalObject* globalObject);

    JSCallbackData m_data;
};

} // namespace WebCore

#endif // JSCustomApplicationInstalledCallback_h

#endif // PLATFORM(ANDROID) && ENABLE(APPLICATION_INSTALLED)
