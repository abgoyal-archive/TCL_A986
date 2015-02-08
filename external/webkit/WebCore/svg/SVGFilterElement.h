

#ifndef SVGFilterElement_h
#define SVGFilterElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "RenderObject.h"
#include "SVGResourceFilter.h"
#include "SVGExternalResourcesRequired.h"
#include "SVGLangSpace.h"
#include "SVGStyledElement.h"
#include "SVGURIReference.h"

namespace WebCore {

    extern char SVGFilterResXIdentifier[];
    extern char SVGFilterResYIdentifier[];

    class SVGResourceFilter;

    class SVGFilterElement : public SVGStyledElement,
                             public SVGURIReference,
                             public SVGLangSpace,
                             public SVGExternalResourcesRequired {
    public:
        SVGFilterElement(const QualifiedName&, Document*);
        virtual ~SVGFilterElement();

        virtual SVGResource* canvasResource(const RenderObject*);

        void setFilterRes(unsigned long filterResX, unsigned long filterResY) const;

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual bool rendererIsNeeded(RenderStyle*) { return false; }

    private:
        DECLARE_ANIMATED_PROPERTY(SVGFilterElement, SVGNames::filterUnitsAttr, int, FilterUnits, filterUnits)
        DECLARE_ANIMATED_PROPERTY(SVGFilterElement, SVGNames::primitiveUnitsAttr, int, PrimitiveUnits, primitiveUnits)
        DECLARE_ANIMATED_PROPERTY(SVGFilterElement, SVGNames::xAttr, SVGLength, X, x)
        DECLARE_ANIMATED_PROPERTY(SVGFilterElement, SVGNames::yAttr, SVGLength, Y, y)
        DECLARE_ANIMATED_PROPERTY(SVGFilterElement, SVGNames::widthAttr, SVGLength, Width, width)
        DECLARE_ANIMATED_PROPERTY(SVGFilterElement, SVGNames::heightAttr, SVGLength, Height, height)
        DECLARE_ANIMATED_PROPERTY_MULTIPLE_WRAPPERS(SVGFilterElement, SVGNames::filterResAttr, SVGFilterResXIdentifier, long, FilterResX, filterResX)
        DECLARE_ANIMATED_PROPERTY_MULTIPLE_WRAPPERS(SVGFilterElement, SVGNames::filterResAttr, SVGFilterResYIdentifier, long, FilterResY, filterResY)

        // SVGURIReference
        DECLARE_ANIMATED_PROPERTY(SVGFilterElement, XLinkNames::hrefAttr, String, Href, href)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGFilterElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)

        mutable RefPtr<SVGResourceFilter> m_filter;

    private:
        friend class SVGResourceFilter;
        FloatRect filterBoundingBox(const FloatRect&) const;
        void buildFilter(const FloatRect& targetRect) const;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
