

#include "config.h"
#include "ErrorConstructor.h"

#include "ErrorPrototype.h"
#include "JSGlobalObject.h"
#include "JSString.h"

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(ErrorConstructor);

ErrorConstructor::ErrorConstructor(ExecState* exec, NonNullPassRefPtr<Structure> structure, ErrorPrototype* errorPrototype)
    : InternalFunction(&exec->globalData(), structure, Identifier(exec, errorPrototype->classInfo()->className))
{
    // ECMA 15.11.3.1 Error.prototype
    putDirectWithoutTransition(exec->propertyNames().prototype, errorPrototype, DontEnum | DontDelete | ReadOnly);
    putDirectWithoutTransition(exec->propertyNames().length, jsNumber(exec, 1), DontDelete | ReadOnly | DontEnum);
}

// ECMA 15.9.3
ErrorInstance* constructError(ExecState* exec, const ArgList& args)
{
    ErrorInstance* obj = new (exec) ErrorInstance(exec->lexicalGlobalObject()->errorStructure());
    if (!args.at(0).isUndefined())
        obj->putDirect(exec->propertyNames().message, jsString(exec, args.at(0).toString(exec)));
    return obj;
}

static JSObject* constructWithErrorConstructor(ExecState* exec, JSObject*, const ArgList& args)
{
    return constructError(exec, args);
}

ConstructType ErrorConstructor::getConstructData(ConstructData& constructData)
{
    constructData.native.function = constructWithErrorConstructor;
    return ConstructTypeHost;
}

// ECMA 15.9.2
static JSValue JSC_HOST_CALL callErrorConstructor(ExecState* exec, JSObject*, JSValue, const ArgList& args)
{
    // "Error()" gives the sames result as "new Error()"
    return constructError(exec, args);
}

CallType ErrorConstructor::getCallData(CallData& callData)
{
    callData.native.function = callErrorConstructor;
    return CallTypeHost;
}

} // namespace JSC
