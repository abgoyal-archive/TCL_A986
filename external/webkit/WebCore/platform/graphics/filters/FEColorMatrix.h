

#ifndef SVGFEColorMatrix_h
#define SVGFEColorMatrix_h

#if ENABLE(FILTERS)
#include "FilterEffect.h"

#include "Filter.h"
#include <wtf/Vector.h>

namespace WebCore {

    enum ColorMatrixType {
        FECOLORMATRIX_TYPE_UNKNOWN          = 0,
        FECOLORMATRIX_TYPE_MATRIX           = 1,
        FECOLORMATRIX_TYPE_SATURATE         = 2,
        FECOLORMATRIX_TYPE_HUEROTATE        = 3,
        FECOLORMATRIX_TYPE_LUMINANCETOALPHA = 4
    };

    class FEColorMatrix : public FilterEffect {
    public:
        static PassRefPtr<FEColorMatrix> create(FilterEffect*, ColorMatrixType, const Vector<float>&);

        ColorMatrixType type() const;
        void setType(ColorMatrixType);

        const Vector<float>& values() const;
        void setValues(const Vector<float>&);

        virtual FloatRect uniteChildEffectSubregions(Filter* filter) { return calculateUnionOfChildEffectSubregions(filter, m_in.get()); }
        void apply(Filter*);
        void dump();

    private:
        FEColorMatrix(FilterEffect*, ColorMatrixType, const Vector<float>&);

        RefPtr<FilterEffect> m_in;
        ColorMatrixType m_type;
        Vector<float> m_values;
    };

} // namespace WebCore

#endif // ENABLE(FILTERS)

#endif // SVGFEColorMatrix_h
