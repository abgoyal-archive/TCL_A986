

#ifndef SVGFEComposite_h
#define SVGFEComposite_h

#if ENABLE(FILTERS)
#include "FilterEffect.h"

#include "PlatformString.h"
#include "Filter.h"

namespace WebCore {

    enum CompositeOperationType {
        FECOMPOSITE_OPERATOR_UNKNOWN    = 0, 
        FECOMPOSITE_OPERATOR_OVER       = 1,
        FECOMPOSITE_OPERATOR_IN         = 2,
        FECOMPOSITE_OPERATOR_OUT        = 3,
        FECOMPOSITE_OPERATOR_ATOP       = 4,
        FECOMPOSITE_OPERATOR_XOR        = 5,
        FECOMPOSITE_OPERATOR_ARITHMETIC = 6
    };

    class FEComposite : public FilterEffect {
    public:
        static PassRefPtr<FEComposite> create(FilterEffect*, FilterEffect*, const CompositeOperationType&,
                const float&, const float&, const float&, const float&);

        CompositeOperationType operation() const;
        void setOperation(CompositeOperationType);

        float k1() const;
        void setK1(float);

        float k2() const;
        void setK2(float);

        float k3() const;
        void setK3(float);

        float k4() const;
        void setK4(float);

        virtual FloatRect uniteChildEffectSubregions(Filter* filter) { return calculateUnionOfChildEffectSubregions(filter, m_in.get(), m_in2.get()); }
        void apply(Filter*);
        void dump();

    private:
        FEComposite(FilterEffect*, FilterEffect*, const CompositeOperationType&,
                const float&, const float&, const float&, const float&);

        RefPtr<FilterEffect> m_in;
        RefPtr<FilterEffect> m_in2;
        CompositeOperationType m_type;
        float m_k1;
        float m_k2;
        float m_k3;
        float m_k4;
    };

} // namespace WebCore

#endif // ENABLE(FILTERS)

#endif // SVGFEComposite_h
