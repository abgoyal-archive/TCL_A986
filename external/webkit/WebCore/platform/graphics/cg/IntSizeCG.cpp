

#include "config.h"
#include "IntSize.h"

#if PLATFORM(CG)

#include <ApplicationServices/ApplicationServices.h>

namespace WebCore {

IntSize::IntSize(const CGSize& s) : m_width(static_cast<int>(s.width)), m_height(static_cast<int>(s.height))
{
}

IntSize::operator CGSize() const
{
    return CGSizeMake(m_width, m_height);
}

}

#endif // PLATFORM(CG)
