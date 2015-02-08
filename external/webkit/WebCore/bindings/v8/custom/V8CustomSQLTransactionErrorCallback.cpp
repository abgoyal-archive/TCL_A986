

#include "config.h"

#if ENABLE(DATABASE)

#include "V8CustomSQLTransactionErrorCallback.h"

#include "Frame.h"
#include "V8CustomVoidCallback.h"
#include "V8SQLError.h"

namespace WebCore {

V8CustomSQLTransactionErrorCallback::V8CustomSQLTransactionErrorCallback(v8::Local<v8::Object> callback, Frame* frame)
    : m_callback(v8::Persistent<v8::Object>::New(callback))
    , m_frame(frame)
{
}

V8CustomSQLTransactionErrorCallback::~V8CustomSQLTransactionErrorCallback()
{
    m_callback.Dispose();
}

void V8CustomSQLTransactionErrorCallback::handleEvent(SQLError* error)
{
    v8::HandleScope handleScope;

    v8::Handle<v8::Context> context = V8Proxy::context(m_frame.get());
    if (context.IsEmpty())
        return;

    v8::Context::Scope scope(context);

    v8::Handle<v8::Value> argv[] = {
        toV8(error)
    };

    // Protect the frame until the callback returns.
    RefPtr<Frame> protector(m_frame);

    bool callbackReturnValue = false;
    invokeCallback(m_callback, 1, argv, callbackReturnValue);
}

} // namespace WebCore

#endif

