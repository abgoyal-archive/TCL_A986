

#include "config.h"
#include "CallData.h"

#include "JSFunction.h"

namespace JSC {

JSValue call(ExecState* exec, JSValue functionObject, CallType callType, const CallData& callData, JSValue thisValue, const ArgList& args)
{
    if (callType == CallTypeHost)
        return callData.native.function(exec, asObject(functionObject), thisValue, args);
    ASSERT(callType == CallTypeJS);
    // FIXME: Can this be done more efficiently using the callData?
    return asFunction(functionObject)->call(exec, thisValue, args);
}

} // namespace JSC
