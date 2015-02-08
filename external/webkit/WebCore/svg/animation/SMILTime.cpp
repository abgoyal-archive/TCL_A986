

#include "config.h"
#if ENABLE(SVG)
#include "SMILTime.h"

#include <float.h>

using namespace WebCore;

const double SMILTime::unresolvedValue = DBL_MAX;
// Just a big value smaller than DBL_MAX. Our times are relative to 0, we don't really need the full range.
const double SMILTime::indefiniteValue = FLT_MAX;    

SMILTime WebCore::operator+(const SMILTime& a, const SMILTime& b)
{
    if (a.isUnresolved() || b.isUnresolved())
        return SMILTime::unresolved();
    if (a.isIndefinite() || b.isIndefinite())
        return SMILTime::indefinite();
    return a.value() + b.value();
}

SMILTime WebCore::operator-(const SMILTime& a, const SMILTime& b)
{
    if (a.isUnresolved() || b.isUnresolved())
        return SMILTime::unresolved();
    if (a.isIndefinite() || b.isIndefinite())
        return SMILTime::indefinite();
    return a.value() - b.value();
}

SMILTime WebCore::operator*(const SMILTime& a,  const SMILTime& b)
{
    if (a.isUnresolved() || b.isUnresolved())
        return SMILTime::unresolved();
    if (a.value() == 0 || b.value() == 0)
        return SMILTime(0);
    if (a.isIndefinite() || b.isIndefinite())
        return SMILTime::indefinite();
    return a.value() * b.value();
}
#endif

