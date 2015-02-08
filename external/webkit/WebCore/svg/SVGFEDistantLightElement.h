

#ifndef SVGFEDistantLightElement_h
#define SVGFEDistantLightElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFELightElement.h"

namespace WebCore {
    class SVGFEDistantLightElement : public SVGFELightElement {
    public:
        SVGFEDistantLightElement(const QualifiedName&, Document*);
        virtual ~SVGFEDistantLightElement();

        virtual PassRefPtr<LightSource> lightSource() const;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
