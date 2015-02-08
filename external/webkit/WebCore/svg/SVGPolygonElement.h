

#ifndef SVGPolygonElement_h
#define SVGPolygonElement_h

#if ENABLE(SVG)
#include "SVGPolyElement.h"

namespace WebCore {

    class SVGPolygonElement : public SVGPolyElement {
    public:
        SVGPolygonElement(const QualifiedName&, Document*);
        virtual ~SVGPolygonElement();

        virtual Path toPathData() const;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
