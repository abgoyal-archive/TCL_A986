

#ifndef SVGSwitchElement_h
#define SVGSwitchElement_h

#if ENABLE(SVG)
#include "SVGExternalResourcesRequired.h"
#include "SVGLangSpace.h"
#include "SVGStyledTransformableElement.h"
#include "SVGTests.h"

namespace WebCore {
    class SVGSwitchElement : public SVGStyledTransformableElement,
                             public SVGTests,
                             public SVGLangSpace,
                             public SVGExternalResourcesRequired {
    public:
        SVGSwitchElement(const QualifiedName&, Document*);
        virtual ~SVGSwitchElement();
        
        virtual bool isValid() const { return SVGTests::isValid(); }

        virtual bool childShouldCreateRenderer(Node*) const;

        virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
        virtual void synchronizeProperty(const QualifiedName&);

    private:
        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGSwitchElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
