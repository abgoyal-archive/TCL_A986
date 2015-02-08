

#ifndef SVGTitleElement_h
#define SVGTitleElement_h
#if ENABLE(SVG)

#include "SVGLangSpace.h"
#include "SVGStyledElement.h"

namespace WebCore {
    class SVGTitleElement : public SVGStyledElement,
                            public SVGLangSpace {
    public:
        SVGTitleElement(const QualifiedName&, Document*);

        virtual void insertedIntoDocument();
        virtual void removedFromDocument();
        virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);

        virtual bool rendererIsNeeded(RenderStyle*) { return false; }
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif

// vim:ts=4:noet
