

#ifndef StyleRareInheritedData_h
#define StyleRareInheritedData_h

#include "AtomicString.h"
#include "Color.h"
#include <wtf/RefCounted.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

struct ShadowData;

// This struct is for rarely used inherited CSS3, CSS2, and WebKit-specific properties.
// By grouping them together, we save space, and only allocate this object when someone
// actually uses one of these properties.
class StyleRareInheritedData : public RefCounted<StyleRareInheritedData> {
public:
    static PassRefPtr<StyleRareInheritedData> create() { return adoptRef(new StyleRareInheritedData); }
    PassRefPtr<StyleRareInheritedData> copy() const { return adoptRef(new StyleRareInheritedData(*this)); }
    ~StyleRareInheritedData();

    bool operator==(const StyleRareInheritedData& o) const;
    bool operator!=(const StyleRareInheritedData& o) const
    {
        return !(*this == o);
    }
    bool shadowDataEquivalent(const StyleRareInheritedData&) const;

    Color textStrokeColor;
    float textStrokeWidth;
    Color textFillColor;
#ifdef ANDROID_CSS_TAP_HIGHLIGHT_COLOR
    Color tapHighlightColor;
#endif

    ShadowData* textShadow; // Our text shadow information for shadowed text drawing.
    AtomicString highlight; // Apple-specific extension for custom highlight rendering.
    unsigned textSecurity : 2; // ETextSecurity
    unsigned userModify : 2; // EUserModify (editing)
    unsigned wordBreak : 2; // EWordBreak
    unsigned wordWrap : 1; // EWordWrap 
    unsigned nbspMode : 1; // ENBSPMode
    unsigned khtmlLineBreak : 1; // EKHTMLLineBreak
    bool textSizeAdjust : 1; // An Apple extension.
    unsigned resize : 2; // EResize
    unsigned userSelect : 1;  // EUserSelect
    unsigned colorSpace : 1; // ColorSpace
    
private:
    StyleRareInheritedData();
    StyleRareInheritedData(const StyleRareInheritedData&);
};

} // namespace WebCore

#endif // StyleRareInheritedData_h
