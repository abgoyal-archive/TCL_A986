

#ifndef SVGFEPointLightElement_h
#define SVGFEPointLightElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFELightElement.h"

namespace WebCore {
    class SVGFEPointLightElement : public SVGFELightElement {
        public:
        SVGFEPointLightElement(const QualifiedName&, Document*);
        virtual ~SVGFEPointLightElement();

        virtual PassRefPtr<LightSource> lightSource() const;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
