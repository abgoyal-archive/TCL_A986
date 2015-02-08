
#if PLATFORM(ANDROID) && ENABLE(APPLICATION_INSTALLED)

#ifndef V8CustomApplicationInstalledCallback_h
#define V8CustomApplicationInstalledCallback_h

#include "ApplicationInstalledCallback.h"
#include <v8.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class Frame;

class V8CustomApplicationInstalledCallback : public ApplicationInstalledCallback {
public:
    static PassRefPtr<V8CustomApplicationInstalledCallback> create(v8::Local<v8::Value> value, Frame* frame)
    {
        ASSERT(value->IsObject());
        return adoptRef(new V8CustomApplicationInstalledCallback(value->ToObject(), frame));
    }
    virtual ~V8CustomApplicationInstalledCallback();

    virtual void handleEvent(bool isInstalled);

private:
    V8CustomApplicationInstalledCallback(v8::Local<v8::Object>, Frame*);

    v8::Persistent<v8::Object> m_callback;
    RefPtr<Frame> m_frame;
};

} // namespace WebCore

#endif // V8CustomApplicationInstalledCallback_h

#endif // PLATFORM(ANDROID) && ENABLE(APPLICATION_INSTALLED)
