

#ifndef RenderSVGRoot_h
#define RenderSVGRoot_h

#if ENABLE(SVG)
#include "RenderBox.h"
#include "FloatRect.h"
#include "SVGRenderSupport.h"

namespace WebCore {

class SVGStyledElement;
class AffineTransform;

class RenderSVGRoot : public RenderBox, protected SVGRenderBase {
public:
    RenderSVGRoot(SVGStyledElement*);

    const RenderObjectChildList* children() const { return &m_children; }
    RenderObjectChildList* children() { return &m_children; }

private:
    virtual RenderObjectChildList* virtualChildren() { return children(); }
    virtual const RenderObjectChildList* virtualChildren() const { return children(); }

    virtual bool isSVGRoot() const { return true; }
    virtual const char* renderName() const { return "RenderSVGRoot"; }

    virtual int lineHeight(bool b, bool isRootLineBox = false) const;
    virtual int baselinePosition(bool b, bool isRootLineBox = false) const;
    virtual void calcPrefWidths();

    virtual void layout();
    virtual void paint(PaintInfo&, int parentX, int parentY);

    virtual const AffineTransform& localToParentTransform() const;

    bool fillContains(const FloatPoint&) const;
    bool strokeContains(const FloatPoint&) const;

    virtual FloatRect objectBoundingBox() const;
    virtual FloatRect strokeBoundingBox() const { return computeContainerBoundingBox(this, true); }
    virtual FloatRect repaintRectInLocalCoordinates() const;

    // FIXME: This override should be removed.
    virtual AffineTransform localTransform() const;

    virtual bool nodeAtPoint(const HitTestRequest&, HitTestResult&, int x, int y, int tx, int ty, HitTestAction);

    virtual void computeRectForRepaint(RenderBoxModelObject* repaintContainer, IntRect& repaintRect, bool fixed);

    virtual void mapLocalToContainer(RenderBoxModelObject* repaintContainer, bool useTransforms, bool fixed, TransformState&) const;

    void calcViewport();

    bool selfWillPaint() const;

    IntSize parentOriginToBorderBox() const;
    IntSize borderOriginToContentBox() const;
    AffineTransform localToRepaintContainerTransform(const IntPoint& parentOriginInContainer) const;
    AffineTransform localToBorderBoxTransform() const;

    RenderObjectChildList m_children;
    FloatSize m_viewportSize;
    mutable AffineTransform m_localToParentTransform;
};

inline RenderSVGRoot* toRenderSVGRoot(RenderObject* object)
{ 
    ASSERT(!object || object->isSVGRoot());
    return static_cast<RenderSVGRoot*>(object);
}

inline const RenderSVGRoot* toRenderSVGRoot(const RenderObject* object)
{ 
    ASSERT(!object || object->isSVGRoot());
    return static_cast<const RenderSVGRoot*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderSVGRoot(const RenderSVGRoot*);

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // RenderSVGRoot_h

// vim:ts=4:noet
