

#ifndef RenderSVGHiddenContainer_h
#define RenderSVGHiddenContainer_h

#if ENABLE(SVG)

#include "RenderSVGContainer.h"

namespace WebCore {
    
    class SVGStyledElement;
    
    // This class is for containers which are never drawn, but do need to support style
    // <defs>, <linearGradient>, <radialGradient> are all good examples
    class RenderSVGHiddenContainer : public RenderSVGContainer {
    public:
        RenderSVGHiddenContainer(SVGStyledElement*);

    private:
        virtual bool isSVGHiddenContainer() const { return true; }

        virtual const char* renderName() const { return "RenderSVGHiddenContainer"; }

        virtual bool requiresLayer() const { return false; }

        virtual void layout();
        virtual void paint(PaintInfo&, int parentX, int parentY);
        
        virtual IntRect clippedOverflowRectForRepaint(RenderBoxModelObject* repaintContainer);
        virtual void absoluteRects(Vector<IntRect>& rects, int tx, int ty);
        virtual void absoluteQuads(Vector<FloatQuad>&);

        virtual FloatRect objectBoundingBox() const;
        virtual FloatRect repaintRectInLocalCoordinates() const;

        virtual bool nodeAtFloatPoint(const HitTestRequest&, HitTestResult&, const FloatPoint& pointInParent, HitTestAction);
    };
}

#endif // ENABLE(SVG)
#endif // RenderSVGHiddenContainer_h
