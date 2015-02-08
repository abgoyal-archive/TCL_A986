

#include "config.h"
#include "JSNumberCell.h"

#if USE(JSVALUE32)

#include "NumberObject.h"
#include "UString.h"

namespace JSC {

JSValue JSNumberCell::toPrimitive(ExecState*, PreferredPrimitiveType) const
{
    return const_cast<JSNumberCell*>(this);
}

bool JSNumberCell::getPrimitiveNumber(ExecState*, double& number, JSValue& value)
{
    number = m_value;
    value = this;
    return true;
}

bool JSNumberCell::toBoolean(ExecState*) const
{
    return m_value < 0.0 || m_value > 0.0; // false for NaN
}

double JSNumberCell::toNumber(ExecState*) const
{
  return m_value;
}

UString JSNumberCell::toString(ExecState*) const
{
    return UString::from(m_value);
}

UString JSNumberCell::toThisString(ExecState*) const
{
    return UString::from(m_value);
}

JSObject* JSNumberCell::toObject(ExecState* exec) const
{
    return constructNumber(exec, const_cast<JSNumberCell*>(this));
}

JSObject* JSNumberCell::toThisObject(ExecState* exec) const
{
    return constructNumber(exec, const_cast<JSNumberCell*>(this));
}

bool JSNumberCell::getUInt32(uint32_t& uint32) const
{
    uint32 = static_cast<uint32_t>(m_value);
    return uint32 == m_value;
}

JSValue JSNumberCell::getJSNumber()
{
    return this;
}

JSValue jsNumberCell(ExecState* exec, double d)
{
    return new (exec) JSNumberCell(exec, d);
}

JSValue jsNumberCell(JSGlobalData* globalData, double d)
{
    return new (globalData) JSNumberCell(globalData, d);
}

} // namespace JSC

#else // USE(JSVALUE32)

// Keep our exported symbols lists happy.
namespace JSC {

JSValue jsNumberCell(ExecState*, double);

JSValue jsNumberCell(ExecState*, double)
{
    ASSERT_NOT_REACHED();
    return JSValue();
}

} // namespace JSC

#endif // USE(JSVALUE32)
