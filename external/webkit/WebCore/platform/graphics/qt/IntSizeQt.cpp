

#include "config.h"

#include <QSize>

#include "IntSize.h"

namespace WebCore {

IntSize::IntSize(const QSize& r)
    : m_width(r.width())
    , m_height(r.height())
{
}

IntSize::operator QSize() const
{
    return QSize(width(), height());
}

}

// vim: ts=4 sw=4 et
