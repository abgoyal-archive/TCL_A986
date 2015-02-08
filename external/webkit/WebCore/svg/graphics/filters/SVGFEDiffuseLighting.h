

#ifndef SVGFEDiffuseLighting_h
#define SVGFEDiffuseLighting_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "Color.h"
#include "FilterEffect.h"
#include "Filter.h"

namespace WebCore {

    class LightSource;

    class FEDiffuseLighting : public FilterEffect {
    public:
        static PassRefPtr<FEDiffuseLighting> create(FilterEffect*, const Color&, const float&, const float&,
                const float&, const float&, PassRefPtr<LightSource>);
        virtual ~FEDiffuseLighting();

        Color lightingColor() const;
        void setLightingColor(const Color&);

        float surfaceScale() const;
        void setSurfaceScale(float);

        float diffuseConstant() const;
        void setDiffuseConstant(float);

        float kernelUnitLengthX() const;
        void setKernelUnitLengthX(float);

        float kernelUnitLengthY() const;
        void setKernelUnitLengthY(float);

        const LightSource* lightSource() const;
        void setLightSource(PassRefPtr<LightSource>);

        virtual FloatRect uniteChildEffectSubregions(Filter* filter) { return calculateUnionOfChildEffectSubregions(filter, m_in.get()); }
        void apply(Filter*);
        void dump();
        TextStream& externalRepresentation(TextStream& ts) const;
        
    private:
        FEDiffuseLighting(FilterEffect*, const Color&, const float&, const float&,
                const float&, const float&, PassRefPtr<LightSource>);

        RefPtr<FilterEffect> m_in;
        Color m_lightingColor;
        float m_surfaceScale;
        float m_diffuseConstant;
        float m_kernelUnitLengthX;
        float m_kernelUnitLengthY;
        RefPtr<LightSource> m_lightSource;
    };

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)

#endif // SVGFEDiffuseLighting_h
