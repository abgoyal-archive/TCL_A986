

#include "config.h"
#include "JSDatabase.h"

#if ENABLE(DATABASE)

#include "DOMWindow.h"
#include "Database.h"
#include "Document.h"
#include "ExceptionCode.h"
#include "JSCustomSQLTransactionCallback.h"
#include "JSCustomSQLTransactionErrorCallback.h"
#include "JSCustomVoidCallback.h"
#include "JSDOMWindowCustom.h"
#include "PlatformString.h"
#include "SQLValue.h"
#include <runtime/JSArray.h>

namespace WebCore {

using namespace JSC;

JSValue JSDatabase::changeVersion(ExecState* exec, const ArgList& args)
{
    String oldVersion = args.at(0).toString(exec);
    String newVersion = args.at(1).toString(exec);

    JSObject* object;
    if (!(object = args.at(2).getObject())) {
        setDOMException(exec, TYPE_MISMATCH_ERR);
        return jsUndefined();
    }
    
    RefPtr<SQLTransactionCallback> callback(JSCustomSQLTransactionCallback::create(object, static_cast<JSDOMGlobalObject*>(exec->dynamicGlobalObject())));
    
    RefPtr<SQLTransactionErrorCallback> errorCallback;
    if (!args.at(3).isNull()) {
        if (!(object = args.at(3).getObject())) {
            setDOMException(exec, TYPE_MISMATCH_ERR);
            return jsUndefined();
        }
        
        errorCallback = JSCustomSQLTransactionErrorCallback::create(object, static_cast<JSDOMGlobalObject*>(exec->dynamicGlobalObject()));
    }
    
    RefPtr<VoidCallback> successCallback;
    if (!args.at(4).isNull()) {
        if (!(object = args.at(4).getObject())) {
            setDOMException(exec, TYPE_MISMATCH_ERR);
            return jsUndefined();
        }

        successCallback = JSCustomVoidCallback::create(object, static_cast<JSDOMGlobalObject*>(exec->dynamicGlobalObject()));
    }

    m_impl->changeVersion(oldVersion, newVersion, callback.release(), errorCallback.release(), successCallback.release());
    
    return jsUndefined();
}

static JSValue createTransaction(ExecState* exec, const ArgList& args, Database* database, JSDOMGlobalObject* globalObject, bool readOnly)
{
    JSObject* object;
    
    if (!(object = args.at(0).getObject())) {
        setDOMException(exec, TYPE_MISMATCH_ERR);
        return jsUndefined();
    }        
     
    RefPtr<SQLTransactionCallback> callback(JSCustomSQLTransactionCallback::create(object, globalObject));
    RefPtr<SQLTransactionErrorCallback> errorCallback;
    
    if (args.size() > 1 && !args.at(1).isNull()) {
        if (!(object = args.at(1).getObject())) {
            setDOMException(exec, TYPE_MISMATCH_ERR);
            return jsUndefined();
        }

        errorCallback = JSCustomSQLTransactionErrorCallback::create(object, globalObject);
    }

    RefPtr<VoidCallback> successCallback;
    if (args.size() > 2 && !args.at(2).isNull()) {
        if (!(object = args.at(2).getObject())) {
            setDOMException(exec, TYPE_MISMATCH_ERR);
            return jsUndefined();
        }

        successCallback = JSCustomVoidCallback::create(object, globalObject);
    }
    
    database->transaction(callback.release(), errorCallback.release(), successCallback.release(), readOnly);
    return jsUndefined();
}

JSValue JSDatabase::transaction(ExecState* exec, const ArgList& args)
{
    return createTransaction(exec, args, m_impl.get(), static_cast<JSDOMGlobalObject*>(exec->dynamicGlobalObject()), false);
}
    
JSValue JSDatabase::readTransaction(ExecState* exec, const ArgList& args)
{
    return createTransaction(exec, args, m_impl.get(), static_cast<JSDOMGlobalObject*>(exec->dynamicGlobalObject()), true);
}

}

#endif // ENABLE(DATABASE)
