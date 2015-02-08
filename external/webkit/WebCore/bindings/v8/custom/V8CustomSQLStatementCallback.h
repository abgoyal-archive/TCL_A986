

#ifndef V8CustomSQLStatementCallback_h
#define V8CustomSQLStatementCallback_h

#if ENABLE(DATABASE)

#include "SQLStatementCallback.h"
#include <v8.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class Frame;

class V8CustomSQLStatementCallback : public SQLStatementCallback {
public:
    static PassRefPtr<V8CustomSQLStatementCallback> create(v8::Local<v8::Value> value, Frame* frame)
    {
        ASSERT(value->IsObject());
        return adoptRef(new V8CustomSQLStatementCallback(value->ToObject(), frame));
    }
    virtual ~V8CustomSQLStatementCallback();

    virtual void handleEvent(SQLTransaction*, SQLResultSet*, bool& raisedException);
private:
    V8CustomSQLStatementCallback(v8::Local<v8::Object>, Frame*);

    v8::Persistent<v8::Object> m_callback;
    RefPtr<Frame> m_frame;
};

} // namespace WebCore

#endif

#endif // V8CustomSQLStatementCallback_h
