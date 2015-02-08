

#include "config.h"
#include "NativeErrorPrototype.h"

#include "ErrorPrototype.h"
#include "JSString.h"
#include "UString.h"

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(NativeErrorPrototype);

NativeErrorPrototype::NativeErrorPrototype(ExecState* exec, NonNullPassRefPtr<Structure> structure, const UString& name, const UString& message)
    : JSObject(structure)
{
    putDirect(exec->propertyNames().name, jsString(exec, name), 0);
    putDirect(exec->propertyNames().message, jsString(exec, message), 0);
}

} // namespace JSC
