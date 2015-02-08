

#ifndef RenderSVGInline_h
#define RenderSVGInline_h

#if ENABLE(SVG)
#include "RenderInline.h"

#include "SVGRenderSupport.h"

namespace WebCore {

class RenderSVGInline : public RenderInline {
public:
    RenderSVGInline(Node*);
    virtual const char* renderName() const { return "RenderSVGInline"; }
    virtual bool requiresLayer() const { return false; }

    // These are shared between RenderSVGTSpan and RenderSVGTextPath
    virtual void absoluteRects(Vector<IntRect>& rects, int tx, int ty);
    virtual void absoluteQuads(Vector<FloatQuad>&);

    virtual FloatRect objectBoundingBox() const { return FloatRect(); }
    virtual FloatRect repaintRectInLocalCoordinates() const { return FloatRect(); }
    
private:
    virtual InlineFlowBox* createInlineFlowBox();
};

}

#endif // ENABLE(SVG)
#endif // !RenderSVGTSpan_H
