

#ifndef V8CustomSQLStatementErrorCallback_h
#define V8CustomSQLStatementErrorCallback_h

#if ENABLE(DATABASE)

#include "SQLStatementErrorCallback.h"

#include "SQLStatementErrorCallback.h"
#include <v8.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class Frame;

class V8CustomSQLStatementErrorCallback : public SQLStatementErrorCallback {
public:
    static PassRefPtr<V8CustomSQLStatementErrorCallback> create(v8::Local<v8::Value> value, Frame* frame)
    {
        ASSERT(value->IsObject());
        return adoptRef(new V8CustomSQLStatementErrorCallback(value->ToObject(), frame));
    }
    virtual ~V8CustomSQLStatementErrorCallback();

    virtual bool handleEvent(SQLTransaction*, SQLError*);
private:
    V8CustomSQLStatementErrorCallback(v8::Local<v8::Object>, Frame*);

    v8::Persistent<v8::Object> m_callback;
    RefPtr<Frame> m_frame;
};

} // namespace WebCore

#endif

#endif // V8CustomSQLStatementErrorCallback_h
