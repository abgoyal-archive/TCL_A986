

#include "config.h"
#include "JSCustomSQLTransactionCallback.h"

#if ENABLE(DATABASE)

#include "Frame.h"
#include "JSCallbackData.h"
#include "JSDOMGlobalObject.h"
#include "JSSQLTransaction.h"
#include "Page.h"
#include "ScriptController.h"
#include <runtime/JSLock.h>
#include <wtf/MainThread.h>
#include <wtf/RefCountedLeakCounter.h>

namespace WebCore {
    
using namespace JSC;
    
#ifndef NDEBUG
static WTF::RefCountedLeakCounter counter("JSCustomSQLTransactionCallback");
#endif

JSCustomSQLTransactionCallback::JSCustomSQLTransactionCallback(JSObject* callback, JSDOMGlobalObject* globalObject)
    : m_data(new JSCallbackData(callback, globalObject))
{
#ifndef NDEBUG
    counter.increment();
#endif
}

JSCustomSQLTransactionCallback::~JSCustomSQLTransactionCallback()
{
    callOnMainThread(JSCallbackData::deleteData, m_data);
#ifndef NDEBUG
    m_data = 0;
    counter.decrement();
#endif
}

void JSCustomSQLTransactionCallback::handleEvent(SQLTransaction* transaction, bool& raisedException)
{
    ASSERT(m_data);

    RefPtr<JSCustomSQLTransactionCallback> protect(this);
        
    JSC::JSLock lock(SilenceAssertionsOnly);
    ExecState* exec = m_data->globalObject()->globalExec();
    MarkedArgumentBuffer args;
    args.append(toJS(exec, deprecatedGlobalObjectForPrototype(exec), transaction));
    m_data->invokeCallback(args, &raisedException);
}
    
}

#endif // ENABLE(DATABASE)
