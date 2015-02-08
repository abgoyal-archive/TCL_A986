

#ifndef RenderSVGContainer_h
#define RenderSVGContainer_h

#if ENABLE(SVG)

#include "RenderSVGModelObject.h"

namespace WebCore {

class SVGElement;

class RenderSVGContainer : public RenderSVGModelObject {
public:
    RenderSVGContainer(SVGStyledElement*);

    const RenderObjectChildList* children() const { return &m_children; }
    RenderObjectChildList* children() { return &m_children; }

    // <marker> uses these methods to only allow drawing children during a special marker draw time
    void setDrawsContents(bool);
    bool drawsContents() const;

    virtual void paint(PaintInfo&, int parentX, int parentY);

protected:
    virtual RenderObjectChildList* virtualChildren() { return children(); }
    virtual const RenderObjectChildList* virtualChildren() const { return children(); }

    virtual bool isSVGContainer() const { return true; }
    virtual const char* renderName() const { return "RenderSVGContainer"; }

    virtual void layout();

    virtual void addFocusRingRects(Vector<IntRect>&, int tx, int ty);

    virtual FloatRect objectBoundingBox() const;
    virtual FloatRect strokeBoundingBox() const;
    virtual FloatRect repaintRectInLocalCoordinates() const;

    virtual bool nodeAtFloatPoint(const HitTestRequest&, HitTestResult&, const FloatPoint& pointInParent, HitTestAction);

    // Allow RenderSVGTransformableContainer to hook in at the right time in layout()
    virtual void calculateLocalTransform() { }

    // Allow RenderSVGViewportContainer to hook in at the right times in layout(), paint() and nodeAtFloatPoint()
    virtual void calcViewport() { }
    virtual void applyViewportClip(PaintInfo&) { }
    virtual bool pointIsInsideViewportClip(const FloatPoint& /*pointInParent*/) { return true; }

    bool selfWillPaint() const;

private:
    RenderObjectChildList m_children;
    bool m_drawsContents : 1;
};
  
inline RenderSVGContainer* toRenderSVGContainer(RenderObject* object)
{
    // Note: isSVGContainer is also true for RenderSVGViewportContainer, which is not derived from this.
    ASSERT(!object || (object->isSVGContainer() && strcmp(object->renderName(), "RenderSVGViewportContainer")));
    return static_cast<RenderSVGContainer*>(object);
}

inline const RenderSVGContainer* toRenderSVGContainer(const RenderObject* object)
{
    // Note: isSVGContainer is also true for RenderSVGViewportContainer, which is not derived from this.
    ASSERT(!object || (object->isSVGContainer() && strcmp(object->renderName(), "RenderSVGViewportContainer")));
    return static_cast<const RenderSVGContainer*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderSVGContainer(const RenderSVGContainer*);

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // RenderSVGContainer_h

// vim:ts=4:noet
