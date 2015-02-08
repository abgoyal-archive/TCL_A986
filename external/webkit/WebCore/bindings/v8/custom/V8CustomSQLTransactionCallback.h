

#ifndef V8CustomSQLTransactionCallback_h
#define V8CustomSQLTransactionCallback_h

#if ENABLE(DATABASE)

#include "SQLTransactionCallback.h"
#include <v8.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class Frame;

class V8CustomSQLTransactionCallback : public SQLTransactionCallback {
public:
    static PassRefPtr<V8CustomSQLTransactionCallback> create(v8::Local<v8::Value> value, Frame* frame)
    {
        ASSERT(value->IsObject());
        return adoptRef(new V8CustomSQLTransactionCallback(value->ToObject(), frame));
    }
    virtual ~V8CustomSQLTransactionCallback();

    virtual void handleEvent(SQLTransaction*, bool& raisedException);
private:
    V8CustomSQLTransactionCallback(v8::Local<v8::Object>, Frame*);

    v8::Persistent<v8::Object> m_callback;
    RefPtr<Frame> m_frame;
};

} // namespace WebCore

#endif

#endif // V8CustomSQLTransactionCallback_h
