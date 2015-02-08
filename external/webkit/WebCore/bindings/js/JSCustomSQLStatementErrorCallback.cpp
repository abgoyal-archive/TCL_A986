

#include "config.h"
#include "JSCustomSQLStatementErrorCallback.h"

#if ENABLE(DATABASE)

#include "Frame.h"
#include "JSCallbackData.h"
#include "JSSQLError.h"
#include "JSSQLTransaction.h"
#include "ScriptController.h"
#include <runtime/JSLock.h>
#include <wtf/MainThread.h>

namespace WebCore {
    
using namespace JSC;
    
JSCustomSQLStatementErrorCallback::JSCustomSQLStatementErrorCallback(JSObject* callback, JSDOMGlobalObject* globalObject)
    : m_data(new JSCallbackData(callback, globalObject))
{
}

JSCustomSQLStatementErrorCallback::~JSCustomSQLStatementErrorCallback()
{
    callOnMainThread(JSCallbackData::deleteData, m_data);
#ifndef NDEBUG
    m_data = 0;
#endif
}

bool JSCustomSQLStatementErrorCallback::handleEvent(SQLTransaction* transaction, SQLError* error)
{
    ASSERT(m_data);
        
    RefPtr<JSCustomSQLStatementErrorCallback> protect(this);
        
    JSC::JSLock lock(SilenceAssertionsOnly);
    ExecState* exec = m_data->globalObject()->globalExec();
    MarkedArgumentBuffer args;
    args.append(toJS(exec, deprecatedGlobalObjectForPrototype(exec), transaction));
    args.append(toJS(exec, deprecatedGlobalObjectForPrototype(exec), error));
    
    bool raisedException = false;
    JSValue result = m_data->invokeCallback(args, &raisedException);
    if (raisedException) {
        // The spec says:
        // "If the error callback returns false, then move on to the next statement..."
        // "Otherwise, the error callback did not return false, or there was no error callback"
        // Therefore an exception and returning true are the same thing - so, return true on an exception
        return true;
    }
    return result.toBoolean(exec);
}

}

#endif // ENABLE(DATABASE)
