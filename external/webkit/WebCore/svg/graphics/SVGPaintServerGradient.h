

#ifndef SVGPaintServerGradient_h
#define SVGPaintServerGradient_h

#if ENABLE(SVG)

#include "AffineTransform.h"
#include "Color.h"
#include "Gradient.h"
#include "GraphicsContext.h"
#include "SVGPaintServer.h"

#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {

    class ImageBuffer;
    class SVGGradientElement;

    typedef std::pair<float, Color> SVGGradientStop;

    class SVGPaintServerGradient : public SVGPaintServer {
    public:
        virtual ~SVGPaintServerGradient();

        void setGradient(PassRefPtr<Gradient>);
        Gradient* gradient() const;

        // Gradient start and end points are percentages when used in boundingBox mode.
        // For instance start point with value (0,0) is top-left and end point with
        // value (100, 100) is bottom-right. BoundingBox mode is enabled by default.
        bool boundingBoxMode() const;
        void setBoundingBoxMode(bool mode = true);

        AffineTransform gradientTransform() const;
        void setGradientTransform(const AffineTransform&);

        void setGradientStops(const Vector<SVGGradientStop>& stops) { m_stops = stops; }
        const Vector<SVGGradientStop>& gradientStops() const { return m_stops; }

        virtual TextStream& externalRepresentation(TextStream&) const;

        virtual bool setup(GraphicsContext*&, const RenderObject*, const RenderStyle*, SVGPaintTargetType, bool isPaintingText) const;
        virtual void teardown(GraphicsContext*&, const RenderObject*, SVGPaintTargetType, bool isPaintingText) const;

    protected:
        SVGPaintServerGradient(const SVGGradientElement* owner);
        
    private:
        Vector<SVGGradientStop> m_stops;
        RefPtr<Gradient> m_gradient;
        bool m_boundingBoxMode;
        AffineTransform m_gradientTransform;
        const SVGGradientElement* m_ownerElement;

#if PLATFORM(CG)
    public:
        mutable GraphicsContext* m_savedContext;
        mutable OwnPtr<ImageBuffer> m_imageBuffer;
#endif
    };

    inline SVGGradientStop makeGradientStop(float offset, const Color& color)
    {
        return std::make_pair(offset, color);
    }

} // namespace WebCore

#endif

#endif // SVGPaintServerGradient_h
