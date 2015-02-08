

#ifndef GapRects_h
#define GapRects_h

#include "IntRect.h"

namespace WebCore {

    struct GapRects {
        const IntRect& left() const { return m_left; }
        const IntRect& center() const { return m_center; }
        const IntRect& right() const { return m_right; }
        
        void uniteLeft(const IntRect& r) { m_left.unite(r); }
        void uniteCenter(const IntRect& r) { m_center.unite(r); }
        void uniteRight(const IntRect& r) { m_right.unite(r); }
        void unite(const GapRects& o) { uniteLeft(o.left()); uniteCenter(o.center()); uniteRight(o.right()); }

        operator IntRect() const
        {
            IntRect result = m_left;
            result.unite(m_center);
            result.unite(m_right);
            return result;
        }

        bool operator==(const GapRects& other)
        {
            return m_left == other.left() && m_center == other.center() && m_right == other.right();
        }
        bool operator!=(const GapRects& other) { return !(*this == other); }

    private:
        IntRect m_left;
        IntRect m_center;
        IntRect m_right;
    };

} // namespace WebCore

#endif // GapRects_h
