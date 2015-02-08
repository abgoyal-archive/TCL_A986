

#ifndef StyleReflection_h
#define StyleReflection_h

#include "CSSReflectionDirection.h"
#include "Length.h"
#include "NinePieceImage.h"
#include <wtf/RefCounted.h>

namespace WebCore {

class StyleReflection : public RefCounted<StyleReflection> {
public:
    static PassRefPtr<StyleReflection> create()
    {
        return adoptRef(new StyleReflection);
    }

    bool operator==(const StyleReflection& o) const
    {
        return m_direction == o.m_direction && m_offset == o.m_offset && m_mask == o.m_mask;
    }
    bool operator!=(const StyleReflection& o) const { return !(*this == o); }

    CSSReflectionDirection direction() const { return m_direction; }
    Length offset() const { return m_offset; }
    const NinePieceImage& mask() const { return m_mask; }

    void setDirection(CSSReflectionDirection dir) { m_direction = dir; }
    void setOffset(const Length& l) { m_offset = l; }
    void setMask(const NinePieceImage& image) { m_mask = image; }

private:
    StyleReflection()
        : m_direction(ReflectionBelow)
        , m_offset(0, Fixed)
    {
    }
    
    CSSReflectionDirection m_direction;
    Length m_offset;
    NinePieceImage m_mask;
};

} // namespace WebCore

#endif // StyleReflection_h
