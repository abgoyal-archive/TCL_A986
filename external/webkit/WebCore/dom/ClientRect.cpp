

#include "config.h"
#include "ClientRect.h"

namespace WebCore {

ClientRect::ClientRect()
{
}

ClientRect::ClientRect(const IntRect& rect)
    : m_rect(rect)
{
}

} // namespace WebCore
