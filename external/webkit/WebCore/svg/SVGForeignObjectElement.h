

#ifndef SVGForeignObjectElement_h
#define SVGForeignObjectElement_h

#if ENABLE(SVG) && ENABLE(SVG_FOREIGN_OBJECT)
#include "SVGTests.h"
#include "SVGLangSpace.h"
#include "SVGURIReference.h"
#include "SVGStyledTransformableElement.h"
#include "SVGExternalResourcesRequired.h"

namespace WebCore {
    class SVGLength;

    class SVGForeignObjectElement : public SVGStyledTransformableElement,
                                    public SVGTests,
                                    public SVGLangSpace,
                                    public SVGExternalResourcesRequired,
                                    public SVGURIReference {
    public:
        SVGForeignObjectElement(const QualifiedName&, Document*);
        virtual ~SVGForeignObjectElement();

        virtual bool isValid() const { return SVGTests::isValid(); }
        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void svgAttributeChanged(const QualifiedName&);
        virtual void synchronizeProperty(const QualifiedName&);

        bool childShouldCreateRenderer(Node*) const;
        virtual RenderObject* createRenderer(RenderArena* arena, RenderStyle* style);

    private:
        DECLARE_ANIMATED_PROPERTY(SVGForeignObjectElement, SVGNames::xAttr, SVGLength, X, x)
        DECLARE_ANIMATED_PROPERTY(SVGForeignObjectElement, SVGNames::yAttr, SVGLength, Y, y)
        DECLARE_ANIMATED_PROPERTY(SVGForeignObjectElement, SVGNames::widthAttr, SVGLength, Width, width)
        DECLARE_ANIMATED_PROPERTY(SVGForeignObjectElement, SVGNames::heightAttr, SVGLength, Height, height)

        // SVGURIReference
        DECLARE_ANIMATED_PROPERTY(SVGForeignObjectElement, XLinkNames::hrefAttr, String, Href, href)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGForeignObjectElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)
    };

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(SVG_FOREIGN_OBJECT)
#endif
