

#ifndef SVGFETile_h
#define SVGFETile_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "FilterEffect.h"
#include "Filter.h"

namespace WebCore {
    
    class FETile : public FilterEffect {
    public:
        static PassRefPtr<FETile> create(FilterEffect*);

        virtual FloatRect uniteChildEffectSubregions(Filter*);
        void apply(Filter*);
        void dump();
        TextStream& externalRepresentation(TextStream& ts) const;
        
    private:
        FETile(FilterEffect*);
        
        RefPtr<FilterEffect> m_in;
    };

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)

#endif // SVGFETile_h
