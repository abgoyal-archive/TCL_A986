


#include "config.h"
#include "JSNotAnObject.h"

#include <wtf/UnusedParam.h>

namespace JSC {

ASSERT_CLASS_FITS_IN_CELL(JSNotAnObject);

// JSValue methods
JSValue JSNotAnObject::toPrimitive(ExecState* exec, PreferredPrimitiveType) const
{
    ASSERT_UNUSED(exec, exec->hadException() && exec->exception() == m_exception);
    return m_exception;
}

bool JSNotAnObject::getPrimitiveNumber(ExecState* exec, double&, JSValue&)
{
    ASSERT_UNUSED(exec, exec->hadException() && exec->exception() == m_exception);
    return false;
}

bool JSNotAnObject::toBoolean(ExecState* exec) const
{
    ASSERT_UNUSED(exec, exec->hadException() && exec->exception() == m_exception);
    return false;
}

double JSNotAnObject::toNumber(ExecState* exec) const
{
    ASSERT_UNUSED(exec, exec->hadException() && exec->exception() == m_exception);
    return NaN;
}

UString JSNotAnObject::toString(ExecState* exec) const
{
    ASSERT_UNUSED(exec, exec->hadException() && exec->exception() == m_exception);
    return "";
}

JSObject* JSNotAnObject::toObject(ExecState* exec) const
{
    ASSERT_UNUSED(exec, exec->hadException() && exec->exception() == m_exception);
    return m_exception;
}

// Marking
void JSNotAnObject::markChildren(MarkStack& markStack)
{
    JSObject::markChildren(markStack);
    markStack.append(m_exception);
}

// JSObject methods
bool JSNotAnObject::getOwnPropertySlot(ExecState* exec, const Identifier&, PropertySlot&)
{
    ASSERT_UNUSED(exec, exec->hadException() && exec->exception() == m_exception);
    return false;
}

bool JSNotAnObject::getOwnPropertySlot(ExecState* exec, unsigned, PropertySlot&)
{
    ASSERT_UNUSED(exec, exec->hadException() && exec->exception() == m_exception);
    return false;
}

bool JSNotAnObject::getOwnPropertyDescriptor(ExecState* exec, const Identifier&, PropertyDescriptor&)
{
    ASSERT_UNUSED(exec, exec->hadException() && exec->exception() == m_exception);
    return false;
}

void JSNotAnObject::put(ExecState* exec, const Identifier& , JSValue, PutPropertySlot&)
{
    ASSERT_UNUSED(exec, exec->hadException() && exec->exception() == m_exception);
}

void JSNotAnObject::put(ExecState* exec, unsigned, JSValue)
{
    ASSERT_UNUSED(exec, exec->hadException() && exec->exception() == m_exception);
}

bool JSNotAnObject::deleteProperty(ExecState* exec, const Identifier&)
{
    ASSERT_UNUSED(exec, exec->hadException() && exec->exception() == m_exception);
    return false;
}

bool JSNotAnObject::deleteProperty(ExecState* exec, unsigned)
{
    ASSERT_UNUSED(exec, exec->hadException() && exec->exception() == m_exception);
    return false;
}

void JSNotAnObject::getOwnPropertyNames(ExecState* exec, PropertyNameArray&, EnumerationMode)
{
    ASSERT_UNUSED(exec, exec->hadException() && exec->exception() == m_exception);
}

} // namespace JSC
