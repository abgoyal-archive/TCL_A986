

#include "config.h"
#include "NativeErrorConstructor.h"

#include "ErrorInstance.h"
#include "JSFunction.h"
#include "JSString.h"
#include "NativeErrorPrototype.h"

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(NativeErrorConstructor);

const ClassInfo NativeErrorConstructor::info = { "Function", &InternalFunction::info, 0, 0 };

NativeErrorConstructor::NativeErrorConstructor(ExecState* exec, NonNullPassRefPtr<Structure> structure, NativeErrorPrototype* nativeErrorPrototype)
    : InternalFunction(&exec->globalData(), structure, Identifier(exec, nativeErrorPrototype->getDirect(exec->propertyNames().name).getString(exec)))
    , m_errorStructure(ErrorInstance::createStructure(nativeErrorPrototype))
{
    putDirect(exec->propertyNames().length, jsNumber(exec, 1), DontDelete | ReadOnly | DontEnum); // ECMA 15.11.7.5
    putDirect(exec->propertyNames().prototype, nativeErrorPrototype, DontDelete | ReadOnly | DontEnum);
}

ErrorInstance* NativeErrorConstructor::construct(ExecState* exec, const ArgList& args)
{
    ErrorInstance* object = new (exec) ErrorInstance(m_errorStructure);
    if (!args.at(0).isUndefined())
        object->putDirect(exec->propertyNames().message, jsString(exec, args.at(0).toString(exec)));
    return object;
}

static JSObject* constructWithNativeErrorConstructor(ExecState* exec, JSObject* constructor, const ArgList& args)
{
    return static_cast<NativeErrorConstructor*>(constructor)->construct(exec, args);
}

ConstructType NativeErrorConstructor::getConstructData(ConstructData& constructData)
{
    constructData.native.function = constructWithNativeErrorConstructor;
    return ConstructTypeHost;
}
    
static JSValue JSC_HOST_CALL callNativeErrorConstructor(ExecState* exec, JSObject* constructor, JSValue, const ArgList& args)
{
    return static_cast<NativeErrorConstructor*>(constructor)->construct(exec, args);
}

CallType NativeErrorConstructor::getCallData(CallData& callData)
{
    callData.native.function = callNativeErrorConstructor;
    return CallTypeHost;
}

} // namespace JSC
