

#ifndef LengthSize_h
#define LengthSize_h

#include "Length.h"

namespace WebCore {

struct LengthSize {
public:
    LengthSize()
    {
    }
    
    LengthSize(Length width, Length height)
        : m_width(width)
        , m_height(height)
    {
    }

    bool operator==(const LengthSize& o) const
    {
        return m_width == o.m_width && m_height == o.m_height;
    }

    void setWidth(Length width) { m_width = width; }
    Length width() const { return m_width; }

    void setHeight(Length height) { m_height = height; }
    Length height() const { return m_height; }

private:
    Length m_width;
    Length m_height;
};

} // namespace WebCore

#endif // LengthSize_h
