

#ifndef SVGFESpotLightElement_h
#define SVGFESpotLightElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFELightElement.h"

namespace WebCore {
    class SVGFESpotLightElement : public SVGFELightElement {
    public:
        SVGFESpotLightElement(const QualifiedName&, Document*);
        virtual ~SVGFESpotLightElement();

        virtual PassRefPtr<LightSource> lightSource() const;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
