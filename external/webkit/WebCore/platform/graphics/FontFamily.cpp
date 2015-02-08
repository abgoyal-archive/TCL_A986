

#include "config.h"
#include "FontFamily.h"

namespace WebCore {

FontFamily::FontFamily(const FontFamily& other)
    : m_family(other.m_family)
    , m_next(other.m_next)
{
}

FontFamily& FontFamily::operator=(const FontFamily& other)
{
    m_family = other.m_family;
    m_next = other.m_next;
    return *this;
}

bool operator==(const FontFamily& a, const FontFamily& b)
{
    if (a.family() != b.family())
        return false;
    const FontFamily* ap;
    const FontFamily* bp;
    for (ap = a.next(), bp = b.next(); ap != bp; ap = ap->next(), bp = bp->next()) {
        if (!ap || !bp)
            return false;
        if (ap->family() != bp->family())
            return false;
    }
    return true;
}

}
