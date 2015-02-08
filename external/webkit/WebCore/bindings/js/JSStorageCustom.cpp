

#include "config.h"
#include "JSStorageCustom.h"

#if ENABLE(DOM_STORAGE)

#include "PlatformString.h"
#include <runtime/PropertyNameArray.h>
#include "Storage.h"

using namespace JSC;

namespace WebCore {

bool JSStorage::canGetItemsForName(ExecState*, Storage* impl, const Identifier& propertyName)
{
    return impl->contains(propertyName);
}

JSValue JSStorage::nameGetter(ExecState* exec, const Identifier& propertyName, const PropertySlot& slot)
{
    JSStorage* thisObj = static_cast<JSStorage*>(asObject(slot.slotBase()));
    return jsStringOrNull(exec, thisObj->impl()->getItem(propertyName));
}

bool JSStorage::deleteProperty(ExecState* exec, const Identifier& propertyName)
{
    // Only perform the custom delete if the object doesn't have a native property by this name.
    // Since hasProperty() would end up calling canGetItemsForName() and be fooled, we need to check
    // the native property slots manually.
    PropertySlot slot;
    if (getStaticValueSlot<JSStorage, Base>(exec, s_info.propHashTable(exec), this, propertyName, slot))
        return false;
        
    JSValue prototype = this->prototype();
    if (prototype.isObject() && asObject(prototype)->hasProperty(exec, propertyName))
        return false;

    m_impl->removeItem(propertyName);
    return true;
}

void JSStorage::getOwnPropertyNames(ExecState* exec, PropertyNameArray& propertyNames, EnumerationMode mode)
{
    unsigned length = m_impl->length();
    for (unsigned i = 0; i < length; ++i)
        propertyNames.add(Identifier(exec, m_impl->key(i)));
        
    Base::getOwnPropertyNames(exec, propertyNames, mode);
}

bool JSStorage::putDelegate(ExecState* exec, const Identifier& propertyName, JSValue value, PutPropertySlot&)
{
    // Only perform the custom put if the object doesn't have a native property by this name.
    // Since hasProperty() would end up calling canGetItemsForName() and be fooled, we need to check
    // the native property slots manually.
    PropertySlot slot;
    if (getStaticValueSlot<JSStorage, Base>(exec, s_info.propHashTable(exec), this, propertyName, slot))
        return false;
        
    JSValue prototype = this->prototype();
    if (prototype.isObject() && asObject(prototype)->hasProperty(exec, propertyName))
        return false;
    
    String stringValue = value.toString(exec);
    if (exec->hadException())
        return true;
    
    ExceptionCode ec = 0;
    impl()->setItem(propertyName, stringValue, ec);
    setDOMException(exec, ec);

    return true;
}

} // namespace WebCore

#endif // ENABLE(DOM_STORAGE)
