


#include "config.h"
#include "IntRect.h"

#include <windows.h>

namespace WebCore {

IntRect::IntRect(const RECT& r)
    : m_location(IntPoint(r.left, r.top)), m_size(IntSize(r.right-r.left, r.bottom-r.top))
{
}

IntRect::operator RECT() const
{
    RECT rect = { x(), y(), right(), bottom() };
    return rect;
}

}
