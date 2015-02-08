

#include "config.h"

#if ENABLE(DATABASE)

#include "V8CustomSQLTransactionCallback.h"

#include "Frame.h"
#include "V8CustomVoidCallback.h"
#include "V8SQLTransaction.h"

namespace WebCore {

V8CustomSQLTransactionCallback::V8CustomSQLTransactionCallback(v8::Local<v8::Object> callback, Frame* frame)
    : m_callback(v8::Persistent<v8::Object>::New(callback))
    , m_frame(frame)
{
}

V8CustomSQLTransactionCallback::~V8CustomSQLTransactionCallback()
{
    m_callback.Dispose();
}


void V8CustomSQLTransactionCallback::handleEvent(SQLTransaction* transaction, bool& raisedException)
{
    v8::HandleScope handleScope;

    v8::Handle<v8::Context> context = V8Proxy::context(m_frame.get());
    if (context.IsEmpty())
        return;

    v8::Context::Scope scope(context);

    v8::Handle<v8::Value> argv[] = {
        toV8(transaction)
    };

    // Protect the frame until the callback returns.
    RefPtr<Frame> protector(m_frame);

    // Step 5: If the callback couldn't be called (e.g. it was null) or if
    // the callback was invoked and raised an exception, jump to the last
    // step (rollback transaction).
    bool callbackReturnValue = false;
    raisedException = invokeCallback(m_callback, 1, argv, callbackReturnValue);
}

} // namespace WebCore

#endif

