

#include "config.h"

#if ENABLE(DATABASE)

#include "V8CustomSQLStatementCallback.h"

#include "Frame.h"
#include "V8CustomVoidCallback.h"
#include "V8SQLResultSet.h"
#include "V8SQLTransaction.h"

namespace WebCore {

V8CustomSQLStatementCallback::V8CustomSQLStatementCallback(v8::Local<v8::Object> callback, Frame* frame)
    : m_callback(v8::Persistent<v8::Object>::New(callback))
    , m_frame(frame)
{
}

V8CustomSQLStatementCallback::~V8CustomSQLStatementCallback()
{
    m_callback.Dispose();
}

void V8CustomSQLStatementCallback::handleEvent(SQLTransaction* transaction, SQLResultSet* resultSet, bool& raisedException)
{
    v8::HandleScope handleScope;

    v8::Handle<v8::Context> context = V8Proxy::context(m_frame.get());
    if (context.IsEmpty())
        return;

    v8::Context::Scope scope(context);

    v8::Handle<v8::Value> argv[] = {
        toV8(transaction),
        toV8(resultSet)
    };

    // Protect the frame until the callback returns.
    RefPtr<Frame> protector(m_frame);

    bool callbackReturnValue = false;
    raisedException = invokeCallback(m_callback, 2, argv, callbackReturnValue);
}

} // namespace WebCore

#endif

