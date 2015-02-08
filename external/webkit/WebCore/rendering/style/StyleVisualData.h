

#ifndef StyleVisualData_h
#define StyleVisualData_h

#include "LengthBox.h"
#include <wtf/RefCounted.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

class StyleVisualData : public RefCounted<StyleVisualData> {
public:
    static PassRefPtr<StyleVisualData> create() { return adoptRef(new StyleVisualData); }
    PassRefPtr<StyleVisualData> copy() const { return adoptRef(new StyleVisualData(*this)); }
    ~StyleVisualData();

    bool operator==(const StyleVisualData& o) const
    {
        return ( clip == o.clip &&
                 hasClip == o.hasClip &&
                 counterIncrement == o.counterIncrement &&
                 counterReset == o.counterReset &&
                 textDecoration == o.textDecoration &&
                 m_zoom == o.m_zoom);
    }
    bool operator!=(const StyleVisualData& o) const { return !(*this == o); }

    LengthBox clip;
    bool hasClip : 1;
    unsigned textDecoration : 4; // Text decorations defined *only* by this element.
    
    short counterIncrement; // ok, so these are not visual mode specific
    short counterReset;     // can't go to inherited, since these are not inherited

    float m_zoom;

private:
    StyleVisualData();
    StyleVisualData(const StyleVisualData&);    
};

} // namespace WebCore

#endif // StyleVisualData_h
