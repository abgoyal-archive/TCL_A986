

#ifndef StyleInheritedData_h
#define StyleInheritedData_h

#include "Color.h"
#include "Font.h"
#include "Length.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class StyleImage;
class CursorList;

class StyleInheritedData : public RefCounted<StyleInheritedData> {
public:
    static PassRefPtr<StyleInheritedData> create() { return adoptRef(new StyleInheritedData); }
    PassRefPtr<StyleInheritedData> copy() const { return adoptRef(new StyleInheritedData(*this)); }
    ~StyleInheritedData();

    bool operator==(const StyleInheritedData& o) const;
    bool operator!=(const StyleInheritedData& o) const
    {
        return !(*this == o);
    }

    Length indent;
    // could be packed in a short but doesn't
    // make a difference currently because of padding
    Length line_height;

    RefPtr<StyleImage> list_style_image;
    RefPtr<CursorList> cursorData;

    Font font;
    Color color;
    
    float m_effectiveZoom;

    short horizontal_border_spacing;
    short vertical_border_spacing;
    
    // Paged media properties.
    short widows;
    short orphans;
    
private:
    StyleInheritedData();
    StyleInheritedData(const StyleInheritedData&);
};

} // namespace WebCore

#endif // StyleInheritedData_h
