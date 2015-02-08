

#include "config.h"

#include "JSStaticScopeObject.h"

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(JSStaticScopeObject);

void JSStaticScopeObject::markChildren(MarkStack& markStack)
{
    JSVariableObject::markChildren(markStack);
    markStack.append(d()->registerStore.jsValue());
}

JSObject* JSStaticScopeObject::toThisObject(ExecState* exec) const
{
    return exec->globalThisValue();
}

void JSStaticScopeObject::put(ExecState*, const Identifier& propertyName, JSValue value, PutPropertySlot&)
{
    if (symbolTablePut(propertyName, value))
        return;
    
    ASSERT_NOT_REACHED();
}

void JSStaticScopeObject::putWithAttributes(ExecState*, const Identifier& propertyName, JSValue value, unsigned attributes)
{
    if (symbolTablePutWithAttributes(propertyName, value, attributes))
        return;
    
    ASSERT_NOT_REACHED();
}

bool JSStaticScopeObject::isDynamicScope() const
{
    return false;
}

JSStaticScopeObject::~JSStaticScopeObject()
{
    ASSERT(d());
    delete d();
}

inline bool JSStaticScopeObject::getOwnPropertySlot(ExecState*, const Identifier& propertyName, PropertySlot& slot)
{
    return symbolTableGet(propertyName, slot);
}

}
