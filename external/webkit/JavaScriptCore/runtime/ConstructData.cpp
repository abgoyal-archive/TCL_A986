

#include "config.h"
#include "ConstructData.h"

#include "JSFunction.h"

namespace JSC {

JSObject* construct(ExecState* exec, JSValue object, ConstructType constructType, const ConstructData& constructData, const ArgList& args)
{
    if (constructType == ConstructTypeHost)
        return constructData.native.function(exec, asObject(object), args);
    ASSERT(constructType == ConstructTypeJS);
    // FIXME: Can this be done more efficiently using the constructData?
    return asFunction(object)->construct(exec, args);
}

} // namespace JSC
