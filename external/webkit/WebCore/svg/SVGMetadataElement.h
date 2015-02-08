

#ifndef SVGMetadataElement_h
#define SVGMetadataElement_h
#if ENABLE(SVG)

#include "SVGElement.h"

namespace WebCore {
    class SVGMetadataElement : public SVGElement {
    public:
        SVGMetadataElement(const QualifiedName&, Document*);
        virtual ~SVGMetadataElement();
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif

// vim:ts=4:noet
