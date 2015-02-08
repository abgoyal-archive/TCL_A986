

#include "config.h"
#include <wtf/Platform.h>
#include "JSCallbackFunction.h"

#include "APIShims.h"
#include "APICast.h"
#include "CodeBlock.h"
#include "JSFunction.h"
#include "FunctionPrototype.h"
#include <runtime/JSGlobalObject.h>
#include <runtime/JSLock.h>
#include <wtf/Vector.h>

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(JSCallbackFunction);

const ClassInfo JSCallbackFunction::info = { "CallbackFunction", &InternalFunction::info, 0, 0 };

JSCallbackFunction::JSCallbackFunction(ExecState* exec, JSObjectCallAsFunctionCallback callback, const Identifier& name)
    : InternalFunction(&exec->globalData(), exec->lexicalGlobalObject()->callbackFunctionStructure(), name)
    , m_callback(callback)
{
}

JSValue JSCallbackFunction::call(ExecState* exec, JSObject* functionObject, JSValue thisValue, const ArgList& args)
{
    JSContextRef execRef = toRef(exec);
    JSObjectRef functionRef = toRef(functionObject);
    JSObjectRef thisObjRef = toRef(thisValue.toThisObject(exec));

    int argumentCount = static_cast<int>(args.size());
    Vector<JSValueRef, 16> arguments(argumentCount);
    for (int i = 0; i < argumentCount; i++)
        arguments[i] = toRef(exec, args.at(i));

    JSValueRef exception = 0;
    JSValueRef result;
    {
        APICallbackShim callbackShim(exec);
        result = static_cast<JSCallbackFunction*>(functionObject)->m_callback(execRef, functionRef, thisObjRef, argumentCount, arguments.data(), &exception);
    }
    if (exception)
        exec->setException(toJS(exec, exception));

    return toJS(exec, result);
}

CallType JSCallbackFunction::getCallData(CallData& callData)
{
    callData.native.function = call;
    return CallTypeHost;
}

} // namespace JSC
