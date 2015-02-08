

#ifndef SVGResourceMarker_h
#define SVGResourceMarker_h

#if ENABLE(SVG)
#include "FloatPoint.h"
#include "FloatRect.h"
#include "RenderObject.h"
#include "SVGResource.h"

namespace WebCore {

    class RenderSVGViewportContainer;
    class AffineTransform;

    class SVGResourceMarker : public SVGResource {
    public:
        static PassRefPtr<SVGResourceMarker> create() { return adoptRef(new SVGResourceMarker); }
        virtual ~SVGResourceMarker();

        RenderSVGViewportContainer* renderer() const { return m_renderer; }
        void setRenderer(RenderSVGViewportContainer* marker) { m_renderer = marker; }

        void setReferencePoint(const FloatPoint& point) { m_referencePoint = point; }
        FloatPoint referencePoint() const { return m_referencePoint; }

        void setAngle(float angle) { m_angle = angle; }
        void setAutoAngle() { m_angle = -1; }
        float angle() const { return m_angle; }

        void setUseStrokeWidth(bool useStrokeWidth = true) { m_useStrokeWidth = useStrokeWidth; }
        bool useStrokeWidth() const { return m_useStrokeWidth; }

        AffineTransform markerTransformation(const FloatPoint& origin, float angle, float strokeWidth) const;
        void draw(RenderObject::PaintInfo&, const AffineTransform&);

        virtual SVGResourceType resourceType() const { return MarkerResourceType; }
        virtual TextStream& externalRepresentation(TextStream&) const;

    private:
        SVGResourceMarker();

        FloatPoint m_referencePoint;
        float m_angle;
        RenderSVGViewportContainer* m_renderer;
        bool m_useStrokeWidth;
    };

    SVGResourceMarker* getMarkerById(Document*, const AtomicString&, const RenderObject*);

} // namespace WebCore

#endif

#endif // SVGResourceMarker_h
