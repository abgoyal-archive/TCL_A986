

#include "config.h"
#include "CounterDirectives.h"

namespace WebCore {

bool operator==(const CounterDirectives& a, const CounterDirectives& b)
{
    if (a.m_reset != b.m_reset || a.m_increment != b.m_increment)
        return false;
    if (a.m_reset && a.m_resetValue != b.m_resetValue)
        return false;
    if (a.m_increment && a.m_incrementValue != b.m_incrementValue)
        return false;
    return true;
}

} // namespace WebCore
