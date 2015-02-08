

#include "config.h"
#include "FloatRect.h"

#include <QRectF>

namespace WebCore {

FloatRect::FloatRect(const QRectF& r)
    : m_location(r.topLeft())
    , m_size(r.width()
    , r.height())
{
}

FloatRect::operator QRectF() const
{
    return QRectF(x(), y(), width(), height());
}

}

// vim: ts=4 sw=4 et
