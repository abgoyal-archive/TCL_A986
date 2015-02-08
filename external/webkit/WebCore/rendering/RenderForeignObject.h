

#ifndef RenderForeignObject_h
#define RenderForeignObject_h
#if ENABLE(SVG) && ENABLE(SVG_FOREIGN_OBJECT)

#include "AffineTransform.h"
#include "FloatPoint.h"
#include "RenderSVGBlock.h"

namespace WebCore {

class SVGForeignObjectElement;

class RenderForeignObject : public RenderSVGBlock {
public:
    RenderForeignObject(SVGForeignObjectElement*);

    virtual const char* renderName() const { return "RenderForeignObject"; }

    virtual void paint(PaintInfo&, int parentX, int parentY);

    virtual const AffineTransform& localToParentTransform() const;

    virtual void computeRectForRepaint(RenderBoxModelObject* repaintContainer, IntRect&, bool fixed = false);
    virtual bool requiresLayer() const { return false; }
    virtual void layout();

    virtual FloatRect objectBoundingBox() const;
    virtual FloatRect strokeBoundingBox() const { return borderBoxRect(); }
    virtual FloatRect repaintRectInLocalCoordinates() const;

    virtual bool nodeAtFloatPoint(const HitTestRequest&, HitTestResult&, const FloatPoint& pointInParent, HitTestAction);
    virtual bool nodeAtPoint(const HitTestRequest&, HitTestResult&, int x, int y, int tx, int ty, HitTestAction);
    virtual bool isSVGForeignObject() const { return true; }

    virtual void mapLocalToContainer(RenderBoxModelObject* repaintContainer, bool fixed , bool useTransforms, TransformState& transformState) const;

 private:
    FloatPoint translationForAttributes() const;

    virtual AffineTransform localTransform() const { return m_localTransform; }

    AffineTransform m_localTransform;
    mutable AffineTransform m_localToParentTransform;
};

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(SVG_FOREIGN_OBJECT)
#endif // RenderForeignObject_h
