

#ifndef RenderPath_h
#define RenderPath_h

#if ENABLE(SVG)
#include "AffineTransform.h"
#include "FloatRect.h"
#include "RenderSVGModelObject.h"
#include "SVGMarkerLayoutInfo.h"

namespace WebCore {

class FloatPoint;
class RenderSVGContainer;
class SVGStyledTransformableElement;

class RenderPath : public RenderSVGModelObject {
public:
    RenderPath(SVGStyledTransformableElement*);

    const Path& path() const { return m_path; }

private:
    // Hit-detection seperated for the fill and the stroke
    bool fillContains(const FloatPoint&, bool requiresFill = true) const;
    bool strokeContains(const FloatPoint&, bool requiresStroke = true) const;

    virtual FloatRect objectBoundingBox() const;
    virtual FloatRect strokeBoundingBox() const;
    virtual FloatRect markerBoundingBox() const;
    virtual FloatRect repaintRectInLocalCoordinates() const;

    virtual const AffineTransform& localToParentTransform() const;

    void setPath(const Path&);

    virtual bool isRenderPath() const { return true; }
    virtual const char* renderName() const { return "RenderPath"; }

    virtual void layout();
    virtual void paint(PaintInfo&, int parentX, int parentY);
    virtual void addFocusRingRects(Vector<IntRect>&, int tx, int ty);

    virtual bool nodeAtFloatPoint(const HitTestRequest&, HitTestResult&, const FloatPoint& pointInParent, HitTestAction);

    void calculateMarkerBoundsIfNeeded() const;

private:
    virtual AffineTransform localTransform() const;

    mutable Path m_path;
    mutable FloatRect m_cachedLocalFillBBox;
    mutable FloatRect m_cachedLocalStrokeBBox;
    mutable FloatRect m_cachedLocalRepaintRect;
    mutable FloatRect m_cachedLocalMarkerBBox;
    mutable SVGMarkerLayoutInfo m_markerLayoutInfo;
    AffineTransform m_localTransform;
};

inline RenderPath* toRenderPath(RenderObject* object)
{
    ASSERT(!object || object->isRenderPath());
    return static_cast<RenderPath*>(object);
}

inline const RenderPath* toRenderPath(const RenderObject* object)
{
    ASSERT(!object || object->isRenderPath());
    return static_cast<const RenderPath*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderPath(const RenderPath*);

}

#endif // ENABLE(SVG)
#endif
