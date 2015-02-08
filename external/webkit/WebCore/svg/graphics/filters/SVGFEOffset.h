

#ifndef SVGFEOffset_h
#define SVGFEOffset_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "FilterEffect.h"
#include "Filter.h"

namespace WebCore {

    class FEOffset : public FilterEffect {
    public:
        static PassRefPtr<FEOffset> create(FilterEffect*, const float&, const float&);

        float dx() const;
        void setDx(float);

        float dy() const;
        void setDy(float);

        virtual FloatRect uniteChildEffectSubregions(Filter* filter) { return calculateUnionOfChildEffectSubregions(filter, m_in.get()); }
        void apply(Filter*);
        void dump();
        TextStream& externalRepresentation(TextStream& ts) const;

    private:
        FEOffset(FilterEffect*, const float&, const float&);

        RefPtr<FilterEffect> m_in;
        float m_dx;
        float m_dy;
    };

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)

#endif // SVGFEOffset_h
