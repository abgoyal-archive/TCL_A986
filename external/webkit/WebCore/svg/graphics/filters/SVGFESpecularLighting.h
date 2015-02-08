

#ifndef SVGFESpecularLighting_h
#define SVGFESpecularLighting_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "Color.h"
#include "FilterEffect.h"
#include "SVGLightSource.h"
#include "Filter.h"

namespace WebCore {

    class FESpecularLighting : public FilterEffect {
    public:
        static PassRefPtr<FESpecularLighting> create(FilterEffect*, const Color&, const float&, const float&,
            const float&, const float&, const float&, PassRefPtr<LightSource>);
        virtual ~FESpecularLighting();

        Color lightingColor() const;
        void setLightingColor(const Color&);

        float surfaceScale() const;
        void setSurfaceScale(float);

        float specularConstant() const;
        void setSpecularConstant(float);

        float specularExponent() const;
        void setSpecularExponent(float);

        float kernelUnitLengthX() const;
        void setKernelUnitLengthX(float);

        float kernelUnitLengthY() const;
        void setKernelUnitLengthY(float);

        const LightSource* lightSource() const;
        void setLightSource(PassRefPtr<LightSource>);

        virtual FloatRect uniteEffectRect(Filter* filter) { return calculateUnionOfChildEffectSubregions(filter, m_in.get()); }
        void apply(Filter*);
        void dump();
        TextStream& externalRepresentation(TextStream& ts) const;

    private:
        FESpecularLighting(FilterEffect*, const Color&, const float&, const float&, const float&,
            const float&, const float&, PassRefPtr<LightSource>);

        RefPtr<FilterEffect> m_in;
        Color m_lightingColor;
        float m_surfaceScale;
        float m_specularConstant;
        float m_specularExponent;
        float m_kernelUnitLengthX;
        float m_kernelUnitLengthY;
        RefPtr<LightSource> m_lightSource;
    };

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)

#endif // SVGFESpecularLighting_h
