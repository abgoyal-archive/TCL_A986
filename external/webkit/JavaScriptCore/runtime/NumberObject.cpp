

#include "config.h"
#include "NumberObject.h"

#include "JSGlobalObject.h"
#include "NumberPrototype.h"

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(NumberObject);

const ClassInfo NumberObject::info = { "Number", 0, 0, 0 };

NumberObject::NumberObject(NonNullPassRefPtr<Structure> structure)
    : JSWrapperObject(structure)
{
}

JSValue NumberObject::getJSNumber()
{
    return internalValue();
}

NumberObject* constructNumber(ExecState* exec, JSValue number)
{
    NumberObject* object = new (exec) NumberObject(exec->lexicalGlobalObject()->numberObjectStructure());
    object->setInternalValue(number);
    return object;
}

} // namespace JSC
