

#include "config.h"
#include "DebuggerActivation.h"

#include "JSActivation.h"

namespace JSC {

DebuggerActivation::DebuggerActivation(JSObject* activation)
    : JSObject(DebuggerActivation::createStructure(jsNull()))
{
    ASSERT(activation);
    ASSERT(activation->isActivationObject());
    m_activation = static_cast<JSActivation*>(activation);
}

void DebuggerActivation::markChildren(MarkStack& markStack)
{
    JSObject::markChildren(markStack);

    if (m_activation)
        markStack.append(m_activation);
}

UString DebuggerActivation::className() const
{
    return m_activation->className();
}

bool DebuggerActivation::getOwnPropertySlot(ExecState* exec, const Identifier& propertyName, PropertySlot& slot)
{
    return m_activation->getOwnPropertySlot(exec, propertyName, slot);
}

void DebuggerActivation::put(ExecState* exec, const Identifier& propertyName, JSValue value, PutPropertySlot& slot)
{
    m_activation->put(exec, propertyName, value, slot);
}

void DebuggerActivation::putWithAttributes(ExecState* exec, const Identifier& propertyName, JSValue value, unsigned attributes)
{
    m_activation->putWithAttributes(exec, propertyName, value, attributes);
}

bool DebuggerActivation::deleteProperty(ExecState* exec, const Identifier& propertyName)
{
    return m_activation->deleteProperty(exec, propertyName);
}

void DebuggerActivation::getOwnPropertyNames(ExecState* exec, PropertyNameArray& propertyNames, EnumerationMode mode)
{
    m_activation->getPropertyNames(exec, propertyNames, mode);
}

bool DebuggerActivation::getOwnPropertyDescriptor(ExecState* exec, const Identifier& propertyName, PropertyDescriptor& descriptor)
{
    return m_activation->getOwnPropertyDescriptor(exec, propertyName, descriptor);
}

void DebuggerActivation::defineGetter(ExecState* exec, const Identifier& propertyName, JSObject* getterFunction, unsigned attributes)
{
    m_activation->defineGetter(exec, propertyName, getterFunction, attributes);
}

void DebuggerActivation::defineSetter(ExecState* exec, const Identifier& propertyName, JSObject* setterFunction, unsigned attributes)
{
    m_activation->defineSetter(exec, propertyName, setterFunction, attributes);
}

JSValue DebuggerActivation::lookupGetter(ExecState* exec, const Identifier& propertyName)
{
    return m_activation->lookupGetter(exec, propertyName);
}

JSValue DebuggerActivation::lookupSetter(ExecState* exec, const Identifier& propertyName)
{
    return m_activation->lookupSetter(exec, propertyName);
}

} // namespace JSC
