

#include "config.h"
#include "DateConversion.h"

#include "CallFrame.h"
#include "UString.h"
#include <wtf/DateMath.h>
#include <wtf/StringExtras.h>

using namespace WTF;

namespace JSC {

double parseDate(ExecState* exec, const UString &date)
{
    if (date == exec->globalData().cachedDateString)
        return exec->globalData().cachedDateStringValue;
    double value = parseDateFromNullTerminatedCharacters(exec, date.UTF8String().c_str());
    exec->globalData().cachedDateString = date;
    exec->globalData().cachedDateStringValue = value;
    return value;
}

void formatDate(const GregorianDateTime &t, DateConversionBuffer& buffer)
{
    snprintf(buffer, DateConversionBufferSize, "%s %s %02d %04d",
        weekdayName[(t.weekDay + 6) % 7],
        monthName[t.month], t.monthDay, t.year + 1900);
}

void formatDateUTCVariant(const GregorianDateTime &t, DateConversionBuffer& buffer)
{
    snprintf(buffer, DateConversionBufferSize, "%s, %02d %s %04d",
        weekdayName[(t.weekDay + 6) % 7],
        t.monthDay, monthName[t.month], t.year + 1900);
}

void formatTime(const GregorianDateTime &t, DateConversionBuffer& buffer)
{
    int offset = abs(gmtoffset(t));
    char timeZoneName[70];
    struct tm gtm = t;
    strftime(timeZoneName, sizeof(timeZoneName), "%Z", &gtm);

    if (timeZoneName[0]) {
        snprintf(buffer, DateConversionBufferSize, "%02d:%02d:%02d GMT%c%02d%02d (%s)",
            t.hour, t.minute, t.second,
            gmtoffset(t) < 0 ? '-' : '+', offset / (60*60), (offset / 60) % 60, timeZoneName);
    } else {
        snprintf(buffer, DateConversionBufferSize, "%02d:%02d:%02d GMT%c%02d%02d",
            t.hour, t.minute, t.second,
            gmtoffset(t) < 0 ? '-' : '+', offset / (60*60), (offset / 60) % 60);
    }
}

void formatTimeUTC(const GregorianDateTime &t, DateConversionBuffer& buffer)
{
    snprintf(buffer, DateConversionBufferSize, "%02d:%02d:%02d GMT", t.hour, t.minute, t.second);
}

} // namespace JSC
