

#include "config.h"
#include "JSCustomSQLTransactionErrorCallback.h"

#if ENABLE(DATABASE)

#include "Frame.h"
#include "JSCallbackData.h"
#include "JSSQLError.h"
#include "ScriptController.h"
#include <runtime/JSLock.h>
#include <wtf/MainThread.h>

namespace WebCore {
    
using namespace JSC;
    
JSCustomSQLTransactionErrorCallback::JSCustomSQLTransactionErrorCallback(JSObject* callback, JSDOMGlobalObject* globalObject)
    : m_data(new JSCallbackData(callback, globalObject))
{
}

JSCustomSQLTransactionErrorCallback::~JSCustomSQLTransactionErrorCallback()
{
    callOnMainThread(JSCallbackData::deleteData, m_data);
#ifndef NDEBUG
    m_data = 0;
#endif
}

void JSCustomSQLTransactionErrorCallback::handleEvent(SQLError* error)
{
    ASSERT(m_data);

    RefPtr<JSCustomSQLTransactionErrorCallback> protect(this);

    JSC::JSLock lock(SilenceAssertionsOnly);
    ExecState* exec = m_data->globalObject()->globalExec();
    MarkedArgumentBuffer args;
    args.append(toJS(exec, deprecatedGlobalObjectForPrototype(exec), error));
    m_data->invokeCallback(args);
}

}

#endif // ENABLE(DATABASE)
