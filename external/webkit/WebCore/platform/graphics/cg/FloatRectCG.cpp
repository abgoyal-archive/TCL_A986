

#include "config.h"
#include "FloatRect.h"

#if PLATFORM(CG)

#include <ApplicationServices/ApplicationServices.h>

namespace WebCore {

FloatRect::FloatRect(const CGRect& r) : m_location(r.origin), m_size(r.size)
{
}

FloatRect::operator CGRect() const
{
    return CGRectMake(x(), y(), width(), height());
}

}

#endif // PLATFORM(CG)
