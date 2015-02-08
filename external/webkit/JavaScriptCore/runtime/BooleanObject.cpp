

#include "config.h"
#include "BooleanObject.h"

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(BooleanObject);

const ClassInfo BooleanObject::info = { "Boolean", 0, 0, 0 };

BooleanObject::BooleanObject(NonNullPassRefPtr<Structure> structure)
    : JSWrapperObject(structure)
{
}

} // namespace JSC
