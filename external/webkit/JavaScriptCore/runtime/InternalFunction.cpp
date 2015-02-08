

#include "config.h"
#include "InternalFunction.h"

#include "FunctionPrototype.h"
#include "JSString.h"

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(InternalFunction);

const ClassInfo InternalFunction::info = { "Function", 0, 0, 0 };

const ClassInfo* InternalFunction::classInfo() const
{
    return &info;
}

InternalFunction::InternalFunction(JSGlobalData* globalData, NonNullPassRefPtr<Structure> structure, const Identifier& name)
    : JSObject(structure)
{
    putDirect(globalData->propertyNames->name, jsString(globalData, name.ustring()), DontDelete | ReadOnly | DontEnum);
}

const UString& InternalFunction::name(ExecState* exec)
{
    return asString(getDirect(exec->globalData().propertyNames->name))->value(exec);
}

const UString InternalFunction::displayName(ExecState* exec)
{
    JSValue displayName = getDirect(exec->globalData().propertyNames->displayName);
    
    if (displayName && isJSString(&exec->globalData(), displayName))
        return asString(displayName)->value(exec);
    
    return UString::null();
}

const UString InternalFunction::calculatedDisplayName(ExecState* exec)
{
    const UString explicitName = displayName(exec);
    
    if (!explicitName.isEmpty())
        return explicitName;
    
    return name(exec);
}

} // namespace JSC
