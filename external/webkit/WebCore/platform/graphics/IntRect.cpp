

#include "config.h"
#include "IntRect.h"

#include "FloatRect.h"
#include <algorithm>

using std::max;
using std::min;

namespace WebCore {

IntRect::IntRect(const FloatRect& r)
    : m_location(IntPoint(static_cast<int>(r.x()), static_cast<int>(r.y())))
    , m_size(IntSize(static_cast<int>(r.width()), static_cast<int>(r.height())))
{
}

bool IntRect::intersects(const IntRect& other) const
{
    // Checking emptiness handles negative widths as well as zero.
    return !isEmpty() && !other.isEmpty()
        && x() < other.right() && other.x() < right()
        && y() < other.bottom() && other.y() < bottom();
}

bool IntRect::contains(const IntRect& other) const
{
    return x() <= other.x() && right() >= other.right()
        && y() <= other.y() && bottom() >= other.bottom();
}

void IntRect::intersect(const IntRect& other)
{
    int l = max(x(), other.x());
    int t = max(y(), other.y());
    int r = min(right(), other.right());
    int b = min(bottom(), other.bottom());

    // Return a clean empty rectangle for non-intersecting cases.
    if (l >= r || t >= b) {
        l = 0;
        t = 0;
        r = 0;
        b = 0;
    }

    m_location.setX(l);
    m_location.setY(t);
    m_size.setWidth(r - l);
    m_size.setHeight(b - t);
}

void IntRect::unite(const IntRect& other)
{
    // Handle empty special cases first.
    if (other.isEmpty())
        return;
    if (isEmpty()) {
        *this = other;
        return;
    }

    int l = min(x(), other.x());
    int t = min(y(), other.y());
    int r = max(right(), other.right());
    int b = max(bottom(), other.bottom());

    m_location.setX(l);
    m_location.setY(t);
    m_size.setWidth(r - l);
    m_size.setHeight(b - t);
}

void IntRect::scale(float s)
{
    m_location.setX((int)(x() * s));
    m_location.setY((int)(y() * s));
    m_size.setWidth((int)(width() * s));
    m_size.setHeight((int)(height() * s));
}

IntRect unionRect(const Vector<IntRect>& rects)
{
    IntRect result;

    size_t count = rects.size();
    for (size_t i = 0; i < count; ++i)
        result.unite(rects[i]);

    return result;
}

} // namespace WebCore
