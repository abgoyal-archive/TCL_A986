

#include "config.h"
#include "JSCustomVoidCallback.h"

#include "Frame.h"
#include "JSCallbackData.h"
#include "JSDOMWindowCustom.h"
#include "ScriptController.h"
#include <runtime/JSLock.h>
#include <wtf/MainThread.h>

namespace WebCore {
    
using namespace JSC;
    
JSCustomVoidCallback::JSCustomVoidCallback(JSObject* callback, JSDOMGlobalObject* globalObject)
    : m_data(new JSCallbackData(callback, globalObject))
{
}

JSCustomVoidCallback::~JSCustomVoidCallback()
{
    callOnMainThread(JSCallbackData::deleteData, m_data);
#ifndef NDEBUG
    m_data = 0;
#endif
}
    
void JSCustomVoidCallback::handleEvent()
{
    ASSERT(m_data);

    RefPtr<JSCustomVoidCallback> protect(this);
        
    JSC::JSLock lock(SilenceAssertionsOnly);
    MarkedArgumentBuffer args;
    m_data->invokeCallback(args);
}

} // namespace WebCore
