

#ifndef RenderSVGViewportContainer_h
#define RenderSVGViewportContainer_h

#if ENABLE(SVG)
#include "RenderSVGContainer.h"

namespace WebCore {

// This is used for non-root <svg> elements and <marker> elements, neither of which are SVGTransformable
// thus we inherit from RenderSVGContainer instead of RenderSVGTransformableContainer
class RenderSVGViewportContainer : public RenderSVGContainer {
public:
    RenderSVGViewportContainer(SVGStyledElement*);

    // Calculates marker boundaries, mapped to the target element's coordinate space
    FloatRect markerBoundaries(const AffineTransform& markerTransformation) const;

    // Generates a transformation matrix usable to render marker content. Handles scaling the marker content
    // acording to SVGs markerUnits="strokeWidth" concept, when a strokeWidth value != -1 is passed in.
    AffineTransform markerContentTransformation(const AffineTransform& contentTransformation, const FloatPoint& origin, float strokeWidth = -1) const;

private:
    virtual bool isSVGContainer() const { return true; }
    virtual const char* renderName() const { return "RenderSVGViewportContainer"; }

    AffineTransform viewportTransform() const;
    virtual const AffineTransform& localToParentTransform() const;

    virtual void calcViewport();

    virtual void applyViewportClip(PaintInfo&);
    virtual bool pointIsInsideViewportClip(const FloatPoint& pointInParent);

    FloatRect m_viewport;
    mutable AffineTransform m_localToParentTransform;
};
  
inline RenderSVGViewportContainer* toRenderSVGViewportContainer(RenderObject* object)
{
    ASSERT(!object || !strcmp(object->renderName(), "RenderSVGViewportContainer"));
    return static_cast<RenderSVGViewportContainer*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderSVGViewportContainer(const RenderSVGViewportContainer*);

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // RenderSVGViewportContainer_h
