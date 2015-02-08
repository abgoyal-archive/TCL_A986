

#include "config.h"
#include "V8CustomApplicationInstalledCallback.h"

#if PLATFORM(ANDROID) && ENABLE(APPLICATION_INSTALLED)

#include "Frame.h"
#include "V8CustomVoidCallback.h"  // For invokeCallback

namespace WebCore {

V8CustomApplicationInstalledCallback::V8CustomApplicationInstalledCallback(v8::Local<v8::Object> callback, Frame* frame)
    : m_callback(v8::Persistent<v8::Object>::New(callback))
    , m_frame(frame)
{
}

V8CustomApplicationInstalledCallback::~V8CustomApplicationInstalledCallback()
{
    m_callback.Dispose();
}

void V8CustomApplicationInstalledCallback::handleEvent(bool isInstalled)
{
    v8::HandleScope handleScope;

    v8::Handle<v8::Context> context = V8Proxy::context(m_frame.get());
    if (context.IsEmpty())
        return;

    v8::Context::Scope scope(context);

    v8::Handle<v8::Value> argv[] = {
        v8::Boolean::New(isInstalled)
    };

    // Protect the frame until the callback returns.
    RefPtr<Frame> protector(m_frame);

    bool callbackReturnValue = false;
    invokeCallback(m_callback, 1, argv, callbackReturnValue);
}

} // namespace WebCore

#endif // PLATFORM(ANDROID) && ENABLE(APPLICATION_INSTALLED)
