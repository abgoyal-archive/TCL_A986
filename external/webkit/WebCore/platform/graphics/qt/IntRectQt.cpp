

#include "config.h"
#include "IntRect.h"

#include <QRect>

namespace WebCore {

IntRect::IntRect(const QRect& r)
    : m_location(r.topLeft())
    , m_size(r.width(), r.height())
{
}

IntRect::operator QRect() const
{
    return QRect(x(), y(), width(), height());
}

}

// vim: ts=4 sw=4 et
