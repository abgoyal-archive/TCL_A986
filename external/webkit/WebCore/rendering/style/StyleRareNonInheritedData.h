

#ifndef StyleRareNonInheritedData_h
#define StyleRareNonInheritedData_h

#include "CounterDirectives.h"
#include "CursorData.h"
#include "DataRef.h"
#include "FillLayer.h"
#include "LineClampValue.h"
#include "NinePieceImage.h"
#include "StyleTransformData.h"
#include <wtf/OwnPtr.h>
#include <wtf/PassRefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

class AnimationList;
class CSSStyleSelector;
class StyleFlexibleBoxData;
class StyleMarqueeData;
class StyleMultiColData;
class StyleReflection;
class StyleTransformData;
struct ContentData;
struct ShadowData;

#if ENABLE(DASHBOARD_SUPPORT)
class StyleDashboardRegion;
#endif

#if ENABLE(XBL)
class BindingURI;
#endif

// This struct is for rarely used non-inherited CSS3, CSS2, and WebKit-specific properties.
// By grouping them together, we save space, and only allocate this object when someone
// actually uses one of these properties.
class StyleRareNonInheritedData : public RefCounted<StyleRareNonInheritedData> {
public:
    static PassRefPtr<StyleRareNonInheritedData> create() { return adoptRef(new StyleRareNonInheritedData); }
    PassRefPtr<StyleRareNonInheritedData> copy() const { return adoptRef(new StyleRareNonInheritedData(*this)); }
    ~StyleRareNonInheritedData();
    
#if ENABLE(XBL)
    bool bindingsEquivalent(const StyleRareNonInheritedData&) const;
#endif

    bool operator==(const StyleRareNonInheritedData&) const;
    bool operator!=(const StyleRareNonInheritedData& o) const { return !(*this == o); }

    bool contentDataEquivalent(const StyleRareNonInheritedData& o) const;
    bool shadowDataEquivalent(const StyleRareNonInheritedData& o) const;
    bool reflectionDataEquivalent(const StyleRareNonInheritedData& o) const;
    bool animationDataEquivalent(const StyleRareNonInheritedData&) const;
    bool transitionDataEquivalent(const StyleRareNonInheritedData&) const;

    LineClampValue lineClamp; // An Apple extension.
#if ENABLE(DASHBOARD_SUPPORT)
    Vector<StyleDashboardRegion> m_dashboardRegions;
#endif
    float opacity; // Whether or not we're transparent.

    DataRef<StyleFlexibleBoxData> flexibleBox; // Flexible box properties 
    DataRef<StyleMarqueeData> marquee; // Marquee properties
    DataRef<StyleMultiColData> m_multiCol; //  CSS3 multicol properties
    DataRef<StyleTransformData> m_transform; // Transform properties (rotate, scale, skew, etc.)

    OwnPtr<ContentData> m_content;
    OwnPtr<CounterDirectiveMap> m_counterDirectives;

    unsigned userDrag : 2; // EUserDrag
    bool textOverflow : 1; // Whether or not lines that spill out should be truncated with "..."
    unsigned marginTopCollapse : 2; // EMarginCollapse
    unsigned marginBottomCollapse : 2; // EMarginCollapse
    unsigned matchNearestMailBlockquoteColor : 1; // EMatchNearestMailBlockquoteColor, FIXME: This property needs to be eliminated. It should never have been added.
    unsigned m_appearance : 6; // EAppearance
    unsigned m_borderFit : 1; // EBorderFit
#if USE(ACCELERATED_COMPOSITING)
    bool m_runningAcceleratedAnimation : 1;
#endif    
    OwnPtr<ShadowData> m_boxShadow;  // For box-shadow decorations.
    
    RefPtr<StyleReflection> m_boxReflect;

    OwnPtr<AnimationList> m_animations;
    OwnPtr<AnimationList> m_transitions;

    FillLayer m_mask;
    NinePieceImage m_maskBoxImage;

    ETransformStyle3D m_transformStyle3D;
    EBackfaceVisibility m_backfaceVisibility;
    float m_perspective;
    Length m_perspectiveOriginX;
    Length m_perspectiveOriginY;

#if ENABLE(XBL)
    OwnPtr<BindingURI> bindingURI; // The XBL binding URI list.
#endif
    
private:
    StyleRareNonInheritedData();
    StyleRareNonInheritedData(const StyleRareNonInheritedData&);
};

} // namespace WebCore

#endif // StyleRareNonInheritedData_h
