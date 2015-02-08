

#ifndef BorderValue_h
#define BorderValue_h

#include "Color.h"
#include "RenderStyleConstants.h"

namespace WebCore {

class BorderValue {
public:
    BorderValue()
        : width(3)
        , m_style(BNONE)
    {
    }

    Color color;
    unsigned width : 12;
    unsigned m_style : 4; // EBorderStyle 

    EBorderStyle style() const { return static_cast<EBorderStyle>(m_style); }
    
    bool nonZero(bool checkStyle = true) const
    {
        return width != 0 && (!checkStyle || m_style != BNONE);
    }

    bool isTransparent() const
    {
        return color.isValid() && color.alpha() == 0;
    }

    bool isVisible(bool checkStyle = true) const
    {
        return nonZero(checkStyle) && !isTransparent() && (!checkStyle || m_style != BHIDDEN);
    }

    bool operator==(const BorderValue& o) const
    {
        return width == o.width && m_style == o.m_style && color == o.color;
    }

    bool operator!=(const BorderValue& o) const
    {
        return !(*this == o);
    }
};

} // namespace WebCore

#endif // BorderValue_h
