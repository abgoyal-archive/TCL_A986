

#ifndef JSCustomSQLTransactionCallback_h
#define JSCustomSQLTransactionCallback_h

#if ENABLE(DATABASE)

#include "SQLTransactionCallback.h"
#include <wtf/PassRefPtr.h>

namespace JSC {
    class JSObject;
}

namespace WebCore {

class Frame;
class JSCallbackData;
class JSDOMGlobalObject;

class JSCustomSQLTransactionCallback : public SQLTransactionCallback {
public:
    static PassRefPtr<JSCustomSQLTransactionCallback> create(JSC::JSObject* callback, JSDOMGlobalObject* globalObject)
    {
        return adoptRef(new JSCustomSQLTransactionCallback(callback, globalObject));
    }

    virtual ~JSCustomSQLTransactionCallback();
    
    virtual void handleEvent(SQLTransaction*, bool& raisedException);

private:
    JSCustomSQLTransactionCallback(JSC::JSObject* callback, JSDOMGlobalObject*);

    JSCallbackData* m_data;
};

}

#endif // ENABLE(DATABASE)

#endif // JSCustomSQLTransactionCallback_h
