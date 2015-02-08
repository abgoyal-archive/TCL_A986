

#ifndef SVGPolylineElement_h
#define SVGPolylineElement_h

#if ENABLE(SVG)
#include "SVGPolyElement.h"

namespace WebCore {

    class SVGPolylineElement : public SVGPolyElement {
    public:
        SVGPolylineElement(const QualifiedName&, Document*);
        virtual ~SVGPolylineElement();

        virtual Path toPathData() const;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
