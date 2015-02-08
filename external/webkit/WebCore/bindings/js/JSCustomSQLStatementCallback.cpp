

#include "config.h"
#include "JSCustomSQLStatementCallback.h"

#if ENABLE(DATABASE)

#include "Frame.h"
#include "ScriptController.h"
#include "JSSQLResultSet.h"
#include "JSSQLTransaction.h"
#include <runtime/JSLock.h>
#include <wtf/MainThread.h>

namespace WebCore {
    
using namespace JSC;
    
JSCustomSQLStatementCallback::JSCustomSQLStatementCallback(JSObject* callback, JSDOMGlobalObject* globalObject)
    : m_data(new JSCallbackData(callback, globalObject))
{
}
    
JSCustomSQLStatementCallback::~JSCustomSQLStatementCallback()
{
    callOnMainThread(JSCallbackData::deleteData, m_data);
#ifndef NDEBUG
    m_data = 0;
#endif
}

void JSCustomSQLStatementCallback::handleEvent(SQLTransaction* transaction, SQLResultSet* resultSet, bool& raisedException)
{
    ASSERT(m_data);

    RefPtr<JSCustomSQLStatementCallback> protect(this);

    JSC::JSLock lock(SilenceAssertionsOnly);
    ExecState* exec = m_data->globalObject()->globalExec();
    MarkedArgumentBuffer args;
    args.append(toJS(exec, deprecatedGlobalObjectForPrototype(exec), transaction));
    args.append(toJS(exec, deprecatedGlobalObjectForPrototype(exec), resultSet));

    m_data->invokeCallback(args, &raisedException);
}

}

#endif // ENABLE(DATABASE)
