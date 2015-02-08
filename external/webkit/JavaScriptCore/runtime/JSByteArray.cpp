

#include "config.h"
#include "JSByteArray.h"

#include "JSGlobalObject.h"
#include "PropertyNameArray.h"

using namespace WTF;

namespace JSC {

const ClassInfo JSByteArray::s_defaultInfo = { "ByteArray", 0, 0, 0 };

JSByteArray::JSByteArray(ExecState* exec, NonNullPassRefPtr<Structure> structure, ByteArray* storage, const JSC::ClassInfo* classInfo)
    : JSObject(structure)
    , m_storage(storage)
    , m_classInfo(classInfo)
{
    putDirect(exec->globalData().propertyNames->length, jsNumber(exec, m_storage->length()), ReadOnly | DontDelete);
}

#if !ASSERT_DISABLED
JSByteArray::~JSByteArray()
{
    ASSERT(vptr() == JSGlobalData::jsByteArrayVPtr);
}
#endif


PassRefPtr<Structure> JSByteArray::createStructure(JSValue prototype)
{
    PassRefPtr<Structure> result = Structure::create(prototype, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount);
    return result;
}

bool JSByteArray::getOwnPropertySlot(ExecState* exec, const Identifier& propertyName, PropertySlot& slot)
{
    bool ok;
    unsigned index = propertyName.toUInt32(&ok, false);
    if (ok && canAccessIndex(index)) {
        slot.setValue(getIndex(exec, index));
        return true;
    }
    return JSObject::getOwnPropertySlot(exec, propertyName, slot);
}

bool JSByteArray::getOwnPropertyDescriptor(ExecState* exec, const Identifier& propertyName, PropertyDescriptor& descriptor)
{
    bool ok;
    unsigned index = propertyName.toUInt32(&ok, false);
    if (ok && canAccessIndex(index)) {
        descriptor.setDescriptor(getIndex(exec, index), DontDelete);
        return true;
    }
    return JSObject::getOwnPropertyDescriptor(exec, propertyName, descriptor);
}

bool JSByteArray::getOwnPropertySlot(ExecState* exec, unsigned propertyName, PropertySlot& slot)
{
    if (canAccessIndex(propertyName)) {
        slot.setValue(getIndex(exec, propertyName));
        return true;
    }
    return JSObject::getOwnPropertySlot(exec, Identifier::from(exec, propertyName), slot);
}

void JSByteArray::put(ExecState* exec, const Identifier& propertyName, JSValue value, PutPropertySlot& slot)
{
    bool ok;
    unsigned index = propertyName.toUInt32(&ok, false);
    if (ok) {
        setIndex(exec, index, value);
        return;
    }
    JSObject::put(exec, propertyName, value, slot);
}

void JSByteArray::put(ExecState* exec, unsigned propertyName, JSValue value)
{
    setIndex(exec, propertyName, value);
}

void JSByteArray::getOwnPropertyNames(ExecState* exec, PropertyNameArray& propertyNames, EnumerationMode mode)
{
    unsigned length = m_storage->length();
    for (unsigned i = 0; i < length; ++i)
        propertyNames.add(Identifier::from(exec, i));
    JSObject::getOwnPropertyNames(exec, propertyNames, mode);
}

}

