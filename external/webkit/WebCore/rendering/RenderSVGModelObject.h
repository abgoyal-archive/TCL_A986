

#ifndef RenderSVGModelObject_h
#define RenderSVGModelObject_h

#if ENABLE(SVG)

#include "RenderObject.h"
#include "SVGRenderSupport.h"

namespace WebCore {

// Most renderers in the SVG rendering tree will inherit from this class
// but not all. (e.g. RenderSVGForeignObject, RenderSVGBlock, RenderSVGImage) thus methods
// required by SVG renders need to be declared on RenderObject, but shared
// logic can go in this class or in SVGRenderBase.

class SVGStyledElement;

class RenderSVGModelObject : public RenderObject, protected SVGRenderBase {
public:
    RenderSVGModelObject(SVGStyledElement*);

    virtual const SVGRenderBase* toSVGRenderBase() const { return this; }

    virtual bool requiresLayer() const { return false; }

    virtual IntRect clippedOverflowRectForRepaint(RenderBoxModelObject* repaintContainer);
    virtual void computeRectForRepaint(RenderBoxModelObject* repaintContainer, IntRect&, bool fixed = false);
    virtual IntRect outlineBoundsForRepaint(RenderBoxModelObject* repaintContainer) const;

    virtual void absoluteRects(Vector<IntRect>& rects, int tx, int ty);
    virtual void absoluteQuads(Vector<FloatQuad>&);

    virtual void mapLocalToContainer(RenderBoxModelObject* repaintContainer, bool useTransforms, bool fixed, TransformState&) const;

private:
    // This method should never be called, SVG uses a different nodeAtPoint method
    bool nodeAtPoint(const HitTestRequest& request, HitTestResult& result, int xInContainer, int yInContainer, int dxParentToContainer, int dyParentToContainer, HitTestAction hitTestAction);
};

}

#endif // ENABLE(SVG)
#endif
