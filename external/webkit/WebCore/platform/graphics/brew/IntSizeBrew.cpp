

#include "config.h"
#include "IntSize.h"

#include <AEE.h>

namespace WebCore {

IntSize::IntSize(const AEESize& size)
    : m_width(size.cx)
    , m_height(size.cy)
{
}

IntSize::operator AEESize() const
{
    AEESize size;
    size.cx = width();
    size.cy = height();
    return size;
}

} // namespace WebCore

