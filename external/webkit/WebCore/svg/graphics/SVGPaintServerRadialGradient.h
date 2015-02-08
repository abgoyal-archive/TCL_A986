

#ifndef SVGPaintServerRadialGradient_h
#define SVGPaintServerRadialGradient_h

#if ENABLE(SVG)

#include "FloatPoint.h"
#include "SVGPaintServerGradient.h"

namespace WebCore {

    class SVGPaintServerRadialGradient : public SVGPaintServerGradient {
    public:
        static PassRefPtr<SVGPaintServerRadialGradient> create(const SVGGradientElement* owner) { return adoptRef(new SVGPaintServerRadialGradient(owner)); }
        virtual ~SVGPaintServerRadialGradient();

        virtual SVGPaintServerType type() const { return RadialGradientPaintServer; }

        FloatPoint gradientCenter() const;
        void setGradientCenter(const FloatPoint&);

        FloatPoint gradientFocal() const;
        void setGradientFocal(const FloatPoint&);

        float gradientRadius() const;
        void setGradientRadius(float);

        virtual TextStream& externalRepresentation(TextStream&) const;

    private:
        SVGPaintServerRadialGradient(const SVGGradientElement* owner);

        float m_radius;
        FloatPoint m_center;
        FloatPoint m_focal;
    };

} // namespace WebCore

#endif

#endif // SVGPaintServerRadialGradient_h
