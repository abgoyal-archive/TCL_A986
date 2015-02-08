

#ifndef JSCustomSQLTransactionErrorCallback_h
#define JSCustomSQLTransactionErrorCallback_h

#if ENABLE(DATABASE)

#include "SQLTransactionErrorCallback.h"
#include "JSDOMGlobalObject.h"
#include <runtime/Protect.h>
#include <wtf/Forward.h>

namespace WebCore {

class JSCallbackData;
class SQLError;

class JSCustomSQLTransactionErrorCallback : public SQLTransactionErrorCallback {
public:
    static PassRefPtr<JSCustomSQLTransactionErrorCallback> create(JSC::JSObject* callback, JSDOMGlobalObject* globalObject)
    {
        return adoptRef(new JSCustomSQLTransactionErrorCallback(callback, globalObject));
    }
    
    virtual ~JSCustomSQLTransactionErrorCallback();
    
    virtual void handleEvent(SQLError*);

private:
    JSCustomSQLTransactionErrorCallback(JSC::JSObject* callback, JSDOMGlobalObject* globalObject);

    JSCallbackData* m_data;
};

}
#endif // ENABLE(DATABASE)

#endif // JSCustomSQLTransactionErrorCallback_h
