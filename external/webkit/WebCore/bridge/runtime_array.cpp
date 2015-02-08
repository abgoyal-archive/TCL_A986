

#include "config.h"
#include "runtime_array.h"

#include <runtime/ArrayPrototype.h>
#include <runtime/Error.h>
#include <runtime/PropertyNameArray.h>
#include "JSDOMBinding.h"

using namespace WebCore;

namespace JSC {

const ClassInfo RuntimeArray::s_info = { "RuntimeArray", &JSArray::info, 0, 0 };

RuntimeArray::RuntimeArray(ExecState* exec, Bindings::Array* array)
    // FIXME: deprecatedGetDOMStructure uses the prototype off of the wrong global object
    // We need to pass in the right global object for "array".
    : JSObject(deprecatedGetDOMStructure<RuntimeArray>(exec))
    , _array(array)
{
}

JSValue RuntimeArray::lengthGetter(ExecState* exec, const Identifier&, const PropertySlot& slot)
{
    RuntimeArray* thisObj = static_cast<RuntimeArray*>(asObject(slot.slotBase()));
    return jsNumber(exec, thisObj->getLength());
}

JSValue RuntimeArray::indexGetter(ExecState* exec, const Identifier&, const PropertySlot& slot)
{
    RuntimeArray* thisObj = static_cast<RuntimeArray*>(asObject(slot.slotBase()));
    return thisObj->getConcreteArray()->valueAt(exec, slot.index());
}

void RuntimeArray::getOwnPropertyNames(ExecState* exec, PropertyNameArray& propertyNames, EnumerationMode mode)
{
    unsigned length = getLength();
    for (unsigned i = 0; i < length; ++i)
        propertyNames.add(Identifier::from(exec, i));

    if (mode == IncludeDontEnumProperties)
        propertyNames.add(exec->propertyNames().length);

    JSObject::getOwnPropertyNames(exec, propertyNames, mode);
}

bool RuntimeArray::getOwnPropertySlot(ExecState* exec, const Identifier& propertyName, PropertySlot& slot)
{
    if (propertyName == exec->propertyNames().length) {
        slot.setCustom(this, lengthGetter);
        return true;
    }
    
    bool ok;
    unsigned index = propertyName.toArrayIndex(&ok);
    if (ok) {
        if (index < getLength()) {
            slot.setCustomIndex(this, index, indexGetter);
            return true;
        }
    }
    
    return JSObject::getOwnPropertySlot(exec, propertyName, slot);
}

bool RuntimeArray::getOwnPropertyDescriptor(ExecState* exec, const Identifier& propertyName, PropertyDescriptor& descriptor)
{
    if (propertyName == exec->propertyNames().length) {
        PropertySlot slot;
        slot.setCustom(this, lengthGetter);
        descriptor.setDescriptor(slot.getValue(exec, propertyName), ReadOnly | DontDelete | DontEnum);
        return true;
    }
    
    bool ok;
    unsigned index = propertyName.toArrayIndex(&ok);
    if (ok) {
        if (index < getLength()) {
            PropertySlot slot;
            slot.setCustomIndex(this, index, indexGetter);
            descriptor.setDescriptor(slot.getValue(exec, propertyName), DontDelete | DontEnum);
            return true;
        }
    }
    
    return JSObject::getOwnPropertyDescriptor(exec, propertyName, descriptor);
}

bool RuntimeArray::getOwnPropertySlot(ExecState *exec, unsigned index, PropertySlot& slot)
{
    if (index < getLength()) {
        slot.setCustomIndex(this, index, indexGetter);
        return true;
    }
    
    return JSObject::getOwnPropertySlot(exec, index, slot);
}

void RuntimeArray::put(ExecState* exec, const Identifier& propertyName, JSValue value, PutPropertySlot& slot)
{
    if (propertyName == exec->propertyNames().length) {
        throwError(exec, RangeError);
        return;
    }
    
    bool ok;
    unsigned index = propertyName.toArrayIndex(&ok);
    if (ok) {
        getConcreteArray()->setValueAt(exec, index, value);
        return;
    }
    
    JSObject::put(exec, propertyName, value, slot);
}

void RuntimeArray::put(ExecState* exec, unsigned index, JSValue value)
{
    if (index >= getLength()) {
        throwError(exec, RangeError);
        return;
    }
    
    getConcreteArray()->setValueAt(exec, index, value);
}

bool RuntimeArray::deleteProperty(ExecState*, const Identifier&)
{
    return false;
}

bool RuntimeArray::deleteProperty(ExecState*, unsigned)
{
    return false;
}

}
