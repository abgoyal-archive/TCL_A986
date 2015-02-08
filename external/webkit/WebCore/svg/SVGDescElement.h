

#ifndef SVGDescElement_h
#define SVGDescElement_h

#if ENABLE(SVG)
#include "SVGLangSpace.h"
#include "SVGStyledElement.h"

namespace WebCore {

    class SVGDescElement : public SVGStyledElement,
                           public SVGLangSpace {
    public:
        SVGDescElement(const QualifiedName&, Document*);
        virtual ~SVGDescElement();

        String description() const;

        virtual bool rendererIsNeeded(RenderStyle*) { return false; }
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
