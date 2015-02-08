

#include "config.h"
#include "IntSize.h"

#include <Size.h>


namespace WebCore {

IntSize::IntSize(const BSize& size)
    : m_width(size.IntegerWidth())
    , m_height(size.IntegerHeight())
{
}

IntSize::operator BSize() const
{
    return BSize(width(), height());
}

} // namespace WebCore

