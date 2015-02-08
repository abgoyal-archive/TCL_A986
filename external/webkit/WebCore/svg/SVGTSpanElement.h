

#ifndef SVGTSpanElement_h
#define SVGTSpanElement_h

#if ENABLE(SVG)
#include "SVGTextPositioningElement.h"

namespace WebCore {

    class SVGTSpanElement : public SVGTextPositioningElement {
    public:
        SVGTSpanElement(const QualifiedName&, Document*);
        virtual ~SVGTSpanElement();
                
        virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
        bool childShouldCreateRenderer(Node*) const;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
