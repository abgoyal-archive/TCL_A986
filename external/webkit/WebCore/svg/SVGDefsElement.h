

#ifndef SVGDefsElement_h
#define SVGDefsElement_h

#if ENABLE(SVG)
#include "SVGExternalResourcesRequired.h"
#include "SVGLangSpace.h"
#include "SVGStyledTransformableElement.h"
#include "SVGTests.h"

namespace WebCore {

    class SVGDefsElement : public SVGStyledTransformableElement,
                           public SVGTests,
                           public SVGLangSpace,
                           public SVGExternalResourcesRequired {
    public:
        SVGDefsElement(const QualifiedName&, Document*);
        virtual ~SVGDefsElement();

        virtual bool isValid() const;

        virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
        virtual void synchronizeProperty(const QualifiedName&);

    private:
        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGDefsElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
