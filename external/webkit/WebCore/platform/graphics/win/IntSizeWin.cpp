

#include "config.h"
#include "IntSize.h"

#include <windows.h>

namespace WebCore {

IntSize::IntSize(const SIZE& s)
    : m_width(s.cx)
    , m_height(s.cy)
{
}

IntSize::operator SIZE() const
{
    SIZE s = {m_width, m_height};
    return s;
}

}
