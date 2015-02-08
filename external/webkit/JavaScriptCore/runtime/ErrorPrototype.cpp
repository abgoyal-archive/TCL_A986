

#include "config.h"
#include "ErrorPrototype.h"

#include "JSFunction.h"
#include "JSString.h"
#include "JSStringBuilder.h"
#include "ObjectPrototype.h"
#include "PrototypeFunction.h"
#include "UString.h"

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(ErrorPrototype);

static JSValue JSC_HOST_CALL errorProtoFuncToString(ExecState*, JSObject*, JSValue, const ArgList&);

// ECMA 15.9.4
ErrorPrototype::ErrorPrototype(ExecState* exec, NonNullPassRefPtr<Structure> structure, Structure* prototypeFunctionStructure)
    : ErrorInstance(structure)
{
    // The constructor will be added later in ErrorConstructor's constructor

    putDirectWithoutTransition(exec->propertyNames().name, jsNontrivialString(exec, "Error"), DontEnum);
    putDirectWithoutTransition(exec->propertyNames().message, jsNontrivialString(exec, "Unknown error"), DontEnum);

    putDirectFunctionWithoutTransition(exec, new (exec) NativeFunctionWrapper(exec, prototypeFunctionStructure, 0, exec->propertyNames().toString, errorProtoFuncToString), DontEnum);
}

JSValue JSC_HOST_CALL errorProtoFuncToString(ExecState* exec, JSObject*, JSValue thisValue, const ArgList&)
{
    JSObject* thisObj = thisValue.toThisObject(exec);
    JSValue name = thisObj->get(exec, exec->propertyNames().name);
    JSValue message = thisObj->get(exec, exec->propertyNames().message);

    // Mozilla-compatible format.

    if (!name.isUndefined()) {
        if (!message.isUndefined())
            return jsMakeNontrivialString(exec, name.toString(exec), ": ", message.toString(exec));
        return jsNontrivialString(exec, name.toString(exec));
    }
    if (!message.isUndefined())
        return jsMakeNontrivialString(exec, "Error: ", message.toString(exec));
    return jsNontrivialString(exec, "Error");
}

} // namespace JSC
