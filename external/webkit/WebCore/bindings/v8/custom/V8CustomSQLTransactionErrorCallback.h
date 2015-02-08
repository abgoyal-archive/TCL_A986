

#ifndef V8CustomSQLTransactionErrorCallback_h
#define V8CustomSQLTransactionErrorCallback_h

#if ENABLE(DATABASE)

#include "SQLTransactionErrorCallback.h"
#include <v8.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class Frame;

class V8CustomSQLTransactionErrorCallback : public SQLTransactionErrorCallback {
public:
    static PassRefPtr<V8CustomSQLTransactionErrorCallback> create(v8::Local<v8::Value> value, Frame* frame)
    {
        ASSERT(value->IsObject());
        return adoptRef(new V8CustomSQLTransactionErrorCallback(value->ToObject(), frame));
    }
    virtual ~V8CustomSQLTransactionErrorCallback();

    virtual void handleEvent(SQLError*);

private:
    V8CustomSQLTransactionErrorCallback(v8::Local<v8::Object>, Frame*);

    v8::Persistent<v8::Object> m_callback;
    RefPtr<Frame> m_frame;
};

} // namespace WebCore

#endif

#endif // V8CustomSQLTransactionErrorCallback_h
