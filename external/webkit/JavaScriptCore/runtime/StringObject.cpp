

#include "config.h"
#include "StringObject.h"

#include "PropertyNameArray.h"

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(StringObject);

const ClassInfo StringObject::info = { "String", 0, 0, 0 };

StringObject::StringObject(ExecState* exec, NonNullPassRefPtr<Structure> structure)
    : JSWrapperObject(structure)
{
    setInternalValue(jsEmptyString(exec));
}

StringObject::StringObject(NonNullPassRefPtr<Structure> structure, JSString* string)
    : JSWrapperObject(structure)
{
    setInternalValue(string);
}

StringObject::StringObject(ExecState* exec, NonNullPassRefPtr<Structure> structure, const UString& string)
    : JSWrapperObject(structure)
{
    setInternalValue(jsString(exec, string));
}

bool StringObject::getOwnPropertySlot(ExecState* exec, const Identifier& propertyName, PropertySlot& slot)
{
    if (internalValue()->getStringPropertySlot(exec, propertyName, slot))
        return true;
    return JSObject::getOwnPropertySlot(exec, propertyName, slot);
}
    
bool StringObject::getOwnPropertySlot(ExecState* exec, unsigned propertyName, PropertySlot& slot)
{
    if (internalValue()->getStringPropertySlot(exec, propertyName, slot))
        return true;    
    return JSObject::getOwnPropertySlot(exec, Identifier::from(exec, propertyName), slot);
}

bool StringObject::getOwnPropertyDescriptor(ExecState* exec, const Identifier& propertyName, PropertyDescriptor& descriptor)
{
    if (internalValue()->getStringPropertyDescriptor(exec, propertyName, descriptor))
        return true;    
    return JSObject::getOwnPropertyDescriptor(exec, propertyName, descriptor);
}

void StringObject::put(ExecState* exec, const Identifier& propertyName, JSValue value, PutPropertySlot& slot)
{
    if (propertyName == exec->propertyNames().length)
        return;
    JSObject::put(exec, propertyName, value, slot);
}

bool StringObject::deleteProperty(ExecState* exec, const Identifier& propertyName)
{
    if (propertyName == exec->propertyNames().length)
        return false;
    bool isStrictUInt32;
    unsigned i = propertyName.toStrictUInt32(&isStrictUInt32);
    if (isStrictUInt32 && internalValue()->canGetIndex(i))
        return false;
    return JSObject::deleteProperty(exec, propertyName);
}

void StringObject::getOwnPropertyNames(ExecState* exec, PropertyNameArray& propertyNames, EnumerationMode mode)
{
    int size = internalValue()->length();
    for (int i = 0; i < size; ++i)
        propertyNames.add(Identifier(exec, UString::from(i)));
    if (mode == IncludeDontEnumProperties)
        propertyNames.add(exec->propertyNames().length);
    return JSObject::getOwnPropertyNames(exec, propertyNames, mode);
}

} // namespace JSC
