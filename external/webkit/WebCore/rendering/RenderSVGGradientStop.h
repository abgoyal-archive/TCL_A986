

#ifndef RenderSVGGradientStop_h
#define RenderSVGGradientStop_h

#if ENABLE(SVG)
#include "RenderObject.h"

namespace WebCore {
    
    class SVGGradientElement;
    class SVGStopElement;
    
    // This class exists mostly so we can hear about gradient stop style changes
    class RenderSVGGradientStop : public RenderObject {
    public:
        RenderSVGGradientStop(SVGStopElement*);
        virtual ~RenderSVGGradientStop();

        virtual const char* renderName() const { return "RenderSVGGradientStop"; }

        virtual void layout();

        // This overrides are needed to prevent ASSERTs on <svg><stop /></svg>
        // RenderObject's default implementations ASSERT_NOT_REACHED()
        // https://bugs.webkit.org/show_bug.cgi?id=20400
        virtual IntRect clippedOverflowRectForRepaint(RenderBoxModelObject*) { return IntRect(); }
        virtual FloatRect objectBoundingBox() const { return FloatRect(); }
        virtual FloatRect repaintRectInLocalCoordinates() const { return FloatRect(); }

    protected:
        virtual void styleDidChange(StyleDifference, const RenderStyle* oldStyle);

    private:
        SVGGradientElement* gradientElement() const;
    };
}

#endif // ENABLE(SVG)
#endif // RenderSVGGradientStop_h
