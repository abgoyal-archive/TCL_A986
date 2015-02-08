
 
#include "config.h"
#include "FloatSize.h"

#if PLATFORM(CG)

#include <ApplicationServices/ApplicationServices.h>

namespace WebCore {

FloatSize::FloatSize(const CGSize& s) : m_width(s.width), m_height(s.height)
{
}

FloatSize::operator CGSize() const
{
    return CGSizeMake(m_width, m_height);
}

}

#endif // PLATFORM(CG)
