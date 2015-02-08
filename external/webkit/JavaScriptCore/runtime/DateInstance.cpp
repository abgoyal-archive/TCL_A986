

#include "config.h"
#include "DateInstance.h"

#include "JSGlobalObject.h"

#include <math.h>
#include <wtf/DateMath.h>
#include <wtf/MathExtras.h>

using namespace WTF;

namespace JSC {

const ClassInfo DateInstance::info = {"Date", 0, 0, 0};

DateInstance::DateInstance(ExecState* exec, NonNullPassRefPtr<Structure> structure)
    : JSWrapperObject(structure)
{
    setInternalValue(jsNaN(exec));
}

DateInstance::DateInstance(ExecState* exec, NonNullPassRefPtr<Structure> structure, double time)
    : JSWrapperObject(structure)
{
    setInternalValue(jsNumber(exec, timeClip(time)));
}

DateInstance::DateInstance(ExecState* exec, double time)
    : JSWrapperObject(exec->lexicalGlobalObject()->dateStructure())
{
    setInternalValue(jsNumber(exec, timeClip(time)));
}

const GregorianDateTime* DateInstance::calculateGregorianDateTime(ExecState* exec) const
{
    double milli = internalNumber();
    if (isnan(milli))
        return 0;

    if (!m_data)
        m_data = exec->globalData().dateInstanceCache.add(milli);

    if (m_data->m_gregorianDateTimeCachedForMS != milli) {
        msToGregorianDateTime(exec, milli, false, m_data->m_cachedGregorianDateTime);
        m_data->m_gregorianDateTimeCachedForMS = milli;
    }
    return &m_data->m_cachedGregorianDateTime;
}

const GregorianDateTime* DateInstance::calculateGregorianDateTimeUTC(ExecState* exec) const
{
    double milli = internalNumber();
    if (isnan(milli))
        return 0;

    if (!m_data)
        m_data = exec->globalData().dateInstanceCache.add(milli);

    if (m_data->m_gregorianDateTimeUTCCachedForMS != milli) {
        msToGregorianDateTime(exec, milli, true, m_data->m_cachedGregorianDateTimeUTC);
        m_data->m_gregorianDateTimeUTCCachedForMS = milli;
    }
    return &m_data->m_cachedGregorianDateTimeUTC;
}

} // namespace JSC
