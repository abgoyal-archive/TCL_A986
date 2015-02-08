

#include "config.h"
#include "GlobalEvalFunction.h"

#include "JSGlobalObject.h"
#include <wtf/Assertions.h>

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(GlobalEvalFunction);

GlobalEvalFunction::GlobalEvalFunction(ExecState* exec, NonNullPassRefPtr<Structure> structure, int len, const Identifier& name, NativeFunction function, JSGlobalObject* cachedGlobalObject)
    : PrototypeFunction(exec, structure, len, name, function)
    , m_cachedGlobalObject(cachedGlobalObject)
{
    ASSERT_ARG(cachedGlobalObject, cachedGlobalObject);
}

void GlobalEvalFunction::markChildren(MarkStack& markStack)
{
    PrototypeFunction::markChildren(markStack);
    markStack.append(m_cachedGlobalObject);
}

} // namespace JSC
