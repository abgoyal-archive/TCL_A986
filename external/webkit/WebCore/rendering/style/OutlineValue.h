

#ifndef OutlineValue_h
#define OutlineValue_h

#include "BorderValue.h"

namespace WebCore {

class OutlineValue : public BorderValue {
public:
    OutlineValue()
        : _offset(0)
        , _auto(false)
    {
    }
    
    bool operator==(const OutlineValue& o) const
    {
        return width == o.width && m_style == o.m_style && color == o.color && _offset == o._offset && _auto == o._auto;
    }
    
    bool operator!=(const OutlineValue& o) const
    {
        return !(*this == o);
    }
    
    int _offset;
    bool _auto;
};

} // namespace WebCore

#endif // OutlineValue_h
