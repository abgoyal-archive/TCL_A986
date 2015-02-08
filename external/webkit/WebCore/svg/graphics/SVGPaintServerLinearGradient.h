

#ifndef SVGPaintServerLinearGradient_h
#define SVGPaintServerLinearGradient_h

#if ENABLE(SVG)

#include "FloatPoint.h"
#include "SVGPaintServerGradient.h"

namespace WebCore {

    class SVGPaintServerLinearGradient : public SVGPaintServerGradient {
    public:
        static PassRefPtr<SVGPaintServerLinearGradient> create(const SVGGradientElement* owner) { return adoptRef(new SVGPaintServerLinearGradient(owner)); }
        virtual ~SVGPaintServerLinearGradient();

        virtual SVGPaintServerType type() const { return LinearGradientPaintServer; }

        FloatPoint gradientStart() const;
        void setGradientStart(const FloatPoint&);

        FloatPoint gradientEnd() const;
        void setGradientEnd(const FloatPoint&);

        virtual TextStream& externalRepresentation(TextStream&) const;

    private:
        SVGPaintServerLinearGradient(const SVGGradientElement* owner);

        FloatPoint m_start;
        FloatPoint m_end;
    };

} // namespace WebCore

#endif

#endif // SVGPaintServerLinearGradient_h
