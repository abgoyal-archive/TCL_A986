

#ifndef JSCustomSQLStatementCallback_h
#define JSCustomSQLStatementCallback_h

#if ENABLE(DATABASE)

#include "JSCallbackData.h"
#include "SQLStatementCallback.h"
#include <wtf/Forward.h>

namespace WebCore {

class SQLResultSet;

class JSCustomSQLStatementCallback : public SQLStatementCallback {
public:
    static PassRefPtr<JSCustomSQLStatementCallback> create(JSC::JSObject* callback, JSDOMGlobalObject* globalObject)
    {
        return adoptRef(new JSCustomSQLStatementCallback(callback, globalObject));
    }
    
    virtual ~JSCustomSQLStatementCallback();

    virtual void handleEvent(SQLTransaction*, SQLResultSet*, bool& raisedException);

private:
    JSCustomSQLStatementCallback(JSC::JSObject* callback, JSDOMGlobalObject*);

    JSCallbackData* m_data;
};

}

#endif // ENABLE(DATABASE)

#endif // JSCustomSQLStatementCallback_h
