

#include "config.h"
#include "JSCallbackData.h"

#include "Document.h"
#include "JSDOMBinding.h"

using namespace JSC;
    
namespace WebCore {

void JSCallbackData::deleteData(void* context)
{
    delete static_cast<JSCallbackData*>(context);
}

JSValue JSCallbackData::invokeCallback(MarkedArgumentBuffer& args, bool* raisedException)
{
    ASSERT(callback());
    ASSERT(globalObject());

    ExecState* exec = globalObject()->globalExec();
    JSValue function = callback()->get(exec, Identifier(exec, "handleEvent"));

    CallData callData;
    CallType callType = function.getCallData(callData);
    if (callType == CallTypeNone) {
        callType = callback()->getCallData(callData);
        if (callType == CallTypeNone)
            return JSValue();
        function = callback();
    }
    
    globalObject()->globalData()->timeoutChecker.start();
    JSValue result = JSC::call(exec, function, callType, callData, callback(), args);
    globalObject()->globalData()->timeoutChecker.stop();

    Document::updateStyleForAllDocuments();

    if (exec->hadException()) {
        reportCurrentException(exec);
        if (raisedException)
            *raisedException = true;
        return result;
    }
    
    return result;
}
    
} // namespace WebCore
