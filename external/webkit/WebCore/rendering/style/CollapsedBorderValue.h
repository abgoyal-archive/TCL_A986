

#ifndef CollapsedBorderValue_h
#define CollapsedBorderValue_h

#include "BorderValue.h"

namespace WebCore {

struct CollapsedBorderValue {
    CollapsedBorderValue()
        : border(0)
        , precedence(BOFF)
    {
    }

    CollapsedBorderValue(const BorderValue* b, EBorderPrecedence p)
        : border(b)
        , precedence(p)
    {
    }

    int width() const { return border && border->nonZero() ? border->width : 0; }
    EBorderStyle style() const { return border ? border->style() : BHIDDEN; }
    bool exists() const { return border; }
    Color color() const { return border ? border->color : Color(); }
    bool isTransparent() const { return border ? border->isTransparent() : true; }
    
    bool operator==(const CollapsedBorderValue& o) const
    {
        if (!border)
            return !o.border;
        if (!o.border)
            return false;
        return *border == *o.border && precedence == o.precedence;
    }
    
    const BorderValue* border;
    EBorderPrecedence precedence;    
};

} // namespace WebCore

#endif // CollapsedBorderValue_h
