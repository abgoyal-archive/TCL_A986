

#include "config.h"
#include "PrototypeFunction.h"

#include "JSGlobalObject.h"
#include <wtf/Assertions.h>

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(PrototypeFunction);

PrototypeFunction::PrototypeFunction(ExecState* exec, int length, const Identifier& name, NativeFunction function)
    : InternalFunction(&exec->globalData(), exec->lexicalGlobalObject()->prototypeFunctionStructure(), name)
    , m_function(function)
{
    ASSERT_ARG(function, function);
    putDirect(exec->propertyNames().length, jsNumber(exec, length), DontDelete | ReadOnly | DontEnum);
}

PrototypeFunction::PrototypeFunction(ExecState* exec, NonNullPassRefPtr<Structure> prototypeFunctionStructure, int length, const Identifier& name, NativeFunction function)
    : InternalFunction(&exec->globalData(), prototypeFunctionStructure, name)
    , m_function(function)
{
    ASSERT_ARG(function, function);
    putDirect(exec->propertyNames().length, jsNumber(exec, length), DontDelete | ReadOnly | DontEnum);
}
    
CallType PrototypeFunction::getCallData(CallData& callData)
{
    callData.native.function = m_function;
    return CallTypeHost;
}

} // namespace JSC
