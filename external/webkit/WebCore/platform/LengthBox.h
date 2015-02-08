

#ifndef LengthBox_h
#define LengthBox_h

#include "Length.h"

namespace WebCore {

struct LengthBox {
    LengthBox()
    {
    }

    LengthBox(LengthType t)
        : m_left(t)
        , m_right(t)
        , m_top(t)
        , m_bottom(t)
    {
    }

    LengthBox(int v)
        : m_left(Length(v, Fixed))
        , m_right(Length(v, Fixed))
        , m_top(Length(v, Fixed))
        , m_bottom(Length(v, Fixed))
    {
    }
    
    LengthBox(int t, int r, int b, int l)
        : m_left(Length(l, Fixed))
        , m_right(Length(r, Fixed))
        , m_top(Length(t, Fixed))
        , m_bottom(Length(b, Fixed))
    {
    }

    Length left() const { return m_left; }
    Length right() const { return m_right; }
    Length top() const { return m_top; }
    Length bottom() const { return m_bottom; }

    bool operator==(const LengthBox& o) const
    {
        return m_left == o.m_left && m_right == o.m_right && m_top == o.m_top && m_bottom == o.m_bottom;
    }

    bool operator!=(const LengthBox& o) const
    {
        return !(*this == o);
    }

    bool nonZero() const
    {
        return !(m_left.isZero() && m_right.isZero() && m_top.isZero() && m_bottom.isZero());
    }

    Length m_left;
    Length m_right;
    Length m_top;
    Length m_bottom;
};

} // namespace WebCore

#endif // LengthBox_h
