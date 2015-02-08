

#ifndef SVGMPathElement_h
#define SVGMPathElement_h

#if ENABLE(SVG)
#include "SVGURIReference.h"
#include "SVGExternalResourcesRequired.h"

namespace WebCore {
    
    class SVGPathElement;
    
    class SVGMPathElement : public SVGElement,
                            public SVGURIReference,
                            public SVGExternalResourcesRequired {
    public:
        SVGMPathElement(const QualifiedName&, Document*);
        virtual ~SVGMPathElement();
        
        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        
        SVGPathElement* pathElement();
        
    private:
        // SVGURIReference
        DECLARE_ANIMATED_PROPERTY(SVGMPathElement, XLinkNames::hrefAttr, String, Href, href)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGMPathElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGMPathElement_h
