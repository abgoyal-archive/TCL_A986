

#include "config.h"
#include "ErrorInstance.h"

namespace JSC {

const ClassInfo ErrorInstance::info = { "Error", 0, 0, 0 };

ErrorInstance::ErrorInstance(NonNullPassRefPtr<Structure> structure)
    : JSObject(structure)
{
}

} // namespace JSC
