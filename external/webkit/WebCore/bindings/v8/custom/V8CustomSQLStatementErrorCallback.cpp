

#include "config.h"

#if ENABLE(DATABASE)

#include "V8CustomSQLStatementErrorCallback.h"

#include "Frame.h"
#include "V8CustomVoidCallback.h"
#include "V8SQLError.h"
#include "V8SQLTransaction.h"

namespace WebCore {

V8CustomSQLStatementErrorCallback::V8CustomSQLStatementErrorCallback(v8::Local<v8::Object> callback, Frame* frame)
    : m_callback(v8::Persistent<v8::Object>::New(callback))
    , m_frame(frame)
{
}

V8CustomSQLStatementErrorCallback::~V8CustomSQLStatementErrorCallback()
{
    m_callback.Dispose();
}

bool V8CustomSQLStatementErrorCallback::handleEvent(SQLTransaction* transaction, SQLError* error)
{
    v8::HandleScope handleScope;

    v8::Handle<v8::Context> context = V8Proxy::context(m_frame.get());
    if (context.IsEmpty())
        return true;

    v8::Context::Scope scope(context);

    v8::Handle<v8::Value> argv[] = {
        toV8(transaction),
        toV8(error)
    };

    // Protect the frame until the callback returns.
    RefPtr<Frame> protector(m_frame);

    bool callbackReturnValue = false;
    // Step 6: If the error callback returns false, then move on to the next
    // statement, if any, or onto the next overall step otherwise. Otherwise,
    // the error callback did not return false, or there was no error callback.
    // Jump to the last step in the overall steps.
    return invokeCallback(m_callback, 2, argv, callbackReturnValue) || callbackReturnValue;
}

} // namespace WebCore

#endif

