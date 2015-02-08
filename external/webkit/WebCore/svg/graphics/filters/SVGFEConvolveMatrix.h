

#ifndef SVGFEConvolveMatrix_h
#define SVGFEConvolveMatrix_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "FilterEffect.h"
#include "FloatPoint.h"
#include "FloatSize.h"
#include "Filter.h"
#include <wtf/Vector.h>

namespace WebCore {

    enum EdgeModeType {
        EDGEMODE_UNKNOWN   = 0,
        EDGEMODE_DUPLICATE = 1,
        EDGEMODE_WRAP      = 2,
        EDGEMODE_NONE      = 3
    };

    class FEConvolveMatrix : public FilterEffect {
    public:
        static PassRefPtr<FEConvolveMatrix> create(FilterEffect*, FilterEffect*, const FloatSize&, 
                const float&, const float&, const FloatSize&, EdgeModeType, const FloatPoint&,
                bool, const Vector<float>&);

        FloatSize kernelSize() const;
        void setKernelSize(FloatSize);

        const Vector<float>& kernel() const;
        void setKernel(const Vector<float>&);

        float divisor() const;
        void setDivisor(float);

        float bias() const;
        void setBias(float);

        FloatSize targetOffset() const;
        void setTargetOffset(FloatSize);

        EdgeModeType edgeMode() const;
        void setEdgeMode(EdgeModeType);

        FloatPoint kernelUnitLength() const;
        void setKernelUnitLength(FloatPoint);

        bool preserveAlpha() const;
        void setPreserveAlpha(bool);

        virtual FloatRect uniteChildEffectSubregions(Filter* filter) { return calculateUnionOfChildEffectSubregions(filter, m_in.get(), m_in2.get()); }
        void apply(Filter*);
        void dump();
        TextStream& externalRepresentation(TextStream& ts) const;

    private:
        FEConvolveMatrix(FilterEffect*, FilterEffect*, const FloatSize&, const float&, const float&,
                const FloatSize&, EdgeModeType, const FloatPoint&, bool, const Vector<float>&);

        RefPtr<FilterEffect> m_in;
        RefPtr<FilterEffect> m_in2;
        FloatSize m_kernelSize;
        float m_divisor;
        float m_bias;
        FloatSize m_targetOffset;
        EdgeModeType m_edgeMode;
        FloatPoint m_kernelUnitLength;
        bool m_preserveAlpha;
        Vector<float> m_kernelMatrix;
    };

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)

#endif // SVGFEConvolveMatrix_h
