

#ifndef SVGFEImage_h
#define SVGFEImage_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "Image.h"
#include "FilterEffect.h"
#include "Filter.h"
#include "SVGPreserveAspectRatio.h"

namespace WebCore {

    class FEImage : public FilterEffect {
    public:
        static PassRefPtr<FEImage> create(RefPtr<Image>, SVGPreserveAspectRatio);

        void apply(Filter*);
        void dump();
        TextStream& externalRepresentation(TextStream& ts) const;
        
    private:
        FEImage(RefPtr<Image>, SVGPreserveAspectRatio);

        RefPtr<Image> m_image;
        SVGPreserveAspectRatio m_preserveAspectRatio;
    };

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)

#endif // SVGFEImage_h
