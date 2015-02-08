

#include "config.h"
#include "V8CustomPositionCallback.h"

#include "Frame.h"
#include "V8CustomVoidCallback.h"  // For invokeCallback
#include "V8Geoposition.h"

namespace WebCore {

V8CustomPositionCallback::V8CustomPositionCallback(v8::Local<v8::Object> callback, Frame* frame)
    : m_callback(v8::Persistent<v8::Object>::New(callback))
    , m_frame(frame)
{
}

V8CustomPositionCallback::~V8CustomPositionCallback()
{
    m_callback.Dispose();
}

void V8CustomPositionCallback::handleEvent(Geoposition* position)
{
    v8::HandleScope handleScope;

    v8::Handle<v8::Context> context = V8Proxy::context(m_frame.get());
    if (context.IsEmpty())
        return;

    v8::Context::Scope scope(context);

    v8::Handle<v8::Value> argv[] = {
        toV8(position)
    };

    // Protect the frame until the callback returns.
    RefPtr<Frame> protector(m_frame);

    bool callbackReturnValue = false;
    invokeCallback(m_callback, 1, argv, callbackReturnValue);
}

} // namespace WebCore
