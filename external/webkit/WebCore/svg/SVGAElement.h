

#ifndef SVGAElement_h
#define SVGAElement_h

#if ENABLE(SVG)
#include "SVGExternalResourcesRequired.h"
#include "SVGLangSpace.h"
#include "SVGStyledTransformableElement.h"
#include "SVGTests.h"
#include "SVGURIReference.h"

namespace WebCore {

    class SVGAElement : public SVGStyledTransformableElement,
                        public SVGURIReference,
                        public SVGTests,
                        public SVGLangSpace,
                        public SVGExternalResourcesRequired {
    public:
        SVGAElement(const QualifiedName&, Document*);
        virtual ~SVGAElement();

        virtual bool isValid() const { return SVGTests::isValid(); }
        
        virtual String title() const;

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);

        virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);

        virtual void defaultEventHandler(Event*);
        
        virtual bool supportsFocus() const;
        virtual bool isMouseFocusable() const;
        virtual bool isKeyboardFocusable(KeyboardEvent*) const;
        virtual bool isFocusable() const;

        virtual bool childShouldCreateRenderer(Node*) const;

    private:
        DECLARE_ANIMATED_PROPERTY(SVGAElement, SVGNames::targetAttr, String, Target, target)

        // SVGURIReference
        DECLARE_ANIMATED_PROPERTY(SVGAElement, XLinkNames::hrefAttr, String, Href, href)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGAElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGAElement_h
