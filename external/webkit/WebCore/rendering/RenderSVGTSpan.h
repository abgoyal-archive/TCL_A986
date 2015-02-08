

#ifndef RenderSVGTSpan_h
#define RenderSVGTSpan_h

#if ENABLE(SVG)
#include "RenderSVGInline.h"

namespace WebCore {
class RenderSVGTSpan : public RenderSVGInline {
public:
    RenderSVGTSpan(Node*);
    virtual const char* renderName() const { return "RenderSVGTSpan"; }

    // FIXME: These are incorrect, but have always behaved this way.
    // These empty implementations prevent us from hitting RenderObject ASSERTS.
    // tspan.getBBox() will be wrong, and repainting for tspans may not work correctly!
    virtual FloatRect objectBoundingBox() const { return FloatRect(); }
    virtual FloatRect repaintRectInLocalCoordinates() const { return FloatRect(); }
};
}

#endif // ENABLE(SVG)
#endif // !RenderSVGTSpan_h
