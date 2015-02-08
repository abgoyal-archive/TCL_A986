

#include "config.h"
#include "JSSQLTransaction.h"

#if ENABLE(DATABASE)

#include "DOMWindow.h"
#include "ExceptionCode.h"
#include "JSCustomSQLStatementCallback.h"
#include "JSCustomSQLStatementErrorCallback.h"
#include "JSDOMWindowCustom.h"
#include "SQLTransaction.h"

using namespace JSC;

namespace WebCore {
    
JSValue JSSQLTransaction::executeSql(ExecState* exec, const ArgList& args)
{
    if (args.isEmpty()) {
        setDOMException(exec, SYNTAX_ERR);
        return jsUndefined();
    }

    String sqlStatement = args.at(0).toString(exec);
    if (exec->hadException())
        return jsUndefined();

    // Now assemble the list of SQL arguments
    Vector<SQLValue> sqlValues;
    if (!args.at(1).isUndefinedOrNull()) {
        JSObject* object = args.at(1).getObject();
        if (!object) {
            setDOMException(exec, TYPE_MISMATCH_ERR);
            return jsUndefined();
        }

        JSValue lengthValue = object->get(exec, exec->propertyNames().length);
        if (exec->hadException())
            return jsUndefined();
        unsigned length = lengthValue.toUInt32(exec);
        if (exec->hadException())
            return jsUndefined();
        
        for (unsigned i = 0 ; i < length; ++i) {
            JSValue value = object->get(exec, i);
            if (exec->hadException())
                return jsUndefined();
            
            if (value.isNull())
                sqlValues.append(SQLValue());
            else if (value.isNumber())
                sqlValues.append(value.uncheckedGetNumber());
            else {
                // Convert the argument to a string and append it
                sqlValues.append(value.toString(exec));
                if (exec->hadException())
                    return jsUndefined();
            }
        }
    }

    RefPtr<SQLStatementCallback> callback;
    if (!args.at(2).isUndefinedOrNull()) {
        JSObject* object = args.at(2).getObject();
        if (!object) {
            setDOMException(exec, TYPE_MISMATCH_ERR);
            return jsUndefined();
        }
        
        callback = JSCustomSQLStatementCallback::create(object, static_cast<JSDOMGlobalObject*>(exec->dynamicGlobalObject()));
    }
    
    RefPtr<SQLStatementErrorCallback> errorCallback;
    if (!args.at(3).isUndefinedOrNull()) {
        JSObject* object = args.at(3).getObject();
        if (!object) {
            setDOMException(exec, TYPE_MISMATCH_ERR);
            return jsUndefined();
        }
        
        errorCallback = JSCustomSQLStatementErrorCallback::create(object, static_cast<JSDOMGlobalObject*>(exec->dynamicGlobalObject()));
    }
    
    ExceptionCode ec = 0;
    m_impl->executeSQL(sqlStatement, sqlValues, callback.release(), errorCallback.release(), ec);
    setDOMException(exec, ec);
    
    return jsUndefined();
}

}

#endif // ENABLE(DATABASE)
