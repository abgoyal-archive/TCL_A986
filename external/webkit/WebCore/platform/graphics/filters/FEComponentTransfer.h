

#ifndef SVGFEComponentTransfer_h
#define SVGFEComponentTransfer_h

#if ENABLE(FILTERS)
#include "FilterEffect.h"

#include "Filter.h"
#include <wtf/Vector.h>

namespace WebCore {

    enum ComponentTransferType {
        FECOMPONENTTRANSFER_TYPE_UNKNOWN  = 0,
        FECOMPONENTTRANSFER_TYPE_IDENTITY = 1,
        FECOMPONENTTRANSFER_TYPE_TABLE    = 2,
        FECOMPONENTTRANSFER_TYPE_DISCRETE = 3,
        FECOMPONENTTRANSFER_TYPE_LINEAR   = 4,
        FECOMPONENTTRANSFER_TYPE_GAMMA    = 5
    };

    struct ComponentTransferFunction {
        ComponentTransferFunction()
            : type(FECOMPONENTTRANSFER_TYPE_UNKNOWN)
            , slope(0.0f)
            , intercept(0.0f)
            , amplitude(0.0f)
            , exponent(0.0f)
            , offset(0.0f)
        {
        }

        ComponentTransferType type;

        float slope;
        float intercept;
        float amplitude;
        float exponent;
        float offset;

        Vector<float> tableValues;
    };

    class FEComponentTransfer : public FilterEffect {
    public:
        static PassRefPtr<FEComponentTransfer> create(FilterEffect*, const ComponentTransferFunction&, 
                const ComponentTransferFunction&, const ComponentTransferFunction&, const ComponentTransferFunction&);

        ComponentTransferFunction redFunction() const;
        void setRedFunction(const ComponentTransferFunction&);

        ComponentTransferFunction greenFunction() const;
        void setGreenFunction(const ComponentTransferFunction&);

        ComponentTransferFunction blueFunction() const;
        void setBlueFunction(const ComponentTransferFunction&);

        ComponentTransferFunction alphaFunction() const;
        void setAlphaFunction(const ComponentTransferFunction&);

        virtual FloatRect uniteChildEffectSubregions(Filter* filter) { return calculateUnionOfChildEffectSubregions(filter, m_in.get()); }
        void apply(Filter*);
        void dump();

    private:
        FEComponentTransfer(FilterEffect*,const ComponentTransferFunction&, const ComponentTransferFunction&,
                const ComponentTransferFunction&, const ComponentTransferFunction&);

        RefPtr<FilterEffect> m_in;
        ComponentTransferFunction m_redFunc;
        ComponentTransferFunction m_greenFunc;
        ComponentTransferFunction m_blueFunc;
        ComponentTransferFunction m_alphaFunc;
    };

} // namespace WebCore

#endif // ENABLE(FILTERS)

#endif // SVGFEComponentTransfer_h
