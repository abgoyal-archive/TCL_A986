

#ifndef SVGFEBlend_h
#define SVGFEBlend_h

#if ENABLE(FILTERS)
#include "FilterEffect.h"

#include "Filter.h"

namespace WebCore {

    enum BlendModeType {
        FEBLEND_MODE_UNKNOWN  = 0,
        FEBLEND_MODE_NORMAL   = 1,
        FEBLEND_MODE_MULTIPLY = 2,
        FEBLEND_MODE_SCREEN   = 3,
        FEBLEND_MODE_DARKEN   = 4,
        FEBLEND_MODE_LIGHTEN  = 5
    };

    class FEBlend : public FilterEffect {
    public:
        static PassRefPtr<FEBlend> create(FilterEffect*, FilterEffect*, BlendModeType);

        FilterEffect* in2() const;
        void setIn2(FilterEffect*);

        BlendModeType blendMode() const;
        void setBlendMode(BlendModeType);

        virtual FloatRect uniteChildEffectSubregions(Filter* filter) { return calculateUnionOfChildEffectSubregions(filter, m_in.get(), m_in2.get()); }
        void apply(Filter*);
        void dump();

    private:
        FEBlend(FilterEffect*, FilterEffect*, BlendModeType);

        RefPtr<FilterEffect> m_in;
        RefPtr<FilterEffect> m_in2;
        BlendModeType m_mode;
    };

} // namespace WebCore

#endif // ENABLE(FILTERS)

#endif // SVGFEBlend_h
