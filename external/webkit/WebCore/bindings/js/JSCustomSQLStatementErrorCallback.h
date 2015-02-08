

#ifndef JSCustomSQLStatementErrorCallback_h
#define JSCustomSQLStatementErrorCallback_h

#if ENABLE(DATABASE)

#include "JSDOMGlobalObject.h"
#include "SQLStatementErrorCallback.h"
#include <runtime/Protect.h>
#include <wtf/Forward.h>

namespace WebCore {

class JSCallbackData;    
class SQLError;
    
class JSCustomSQLStatementErrorCallback : public SQLStatementErrorCallback {
public:
    static PassRefPtr<JSCustomSQLStatementErrorCallback> create(JSC::JSObject* callback, JSDOMGlobalObject* globalObject)
    {
        return adoptRef(new JSCustomSQLStatementErrorCallback(callback, globalObject));
    }

    virtual ~JSCustomSQLStatementErrorCallback();

    virtual bool handleEvent(SQLTransaction*, SQLError*);

private:
    JSCustomSQLStatementErrorCallback(JSC::JSObject* callback, JSDOMGlobalObject*);

    JSCallbackData* m_data;
};
    
}

#endif // ENABLE(DATABASE)

#endif // JSCustomSQLStatementErrorCallback_h

