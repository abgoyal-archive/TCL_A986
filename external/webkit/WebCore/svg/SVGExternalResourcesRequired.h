

#ifndef SVGExternalResourcesRequired_h
#define SVGExternalResourcesRequired_h

#if ENABLE(SVG)
#include "SVGElement.h"
#include "SVGNames.h"

namespace WebCore {

    class MappedAttribute;

    // Notes on a SVG 1.1 spec discrepancy:
    // The SVG DOM defines the attribute externalResourcesRequired as being of type SVGAnimatedBoolean, whereas the 
    // SVG language definition says that externalResourcesRequired is not animated. Because the SVG language definition
    // states that externalResourcesRequired cannot be animated, the animVal will always be the same as the baseVal.
    // FIXME: When implementing animVal support, make sure that animVal==baseVal for externalResourcesRequired
    class SVGExternalResourcesRequired {
    public:
        SVGExternalResourcesRequired();
        virtual ~SVGExternalResourcesRequired();

        bool parseMappedAttribute(MappedAttribute*);
        bool isKnownAttribute(const QualifiedName&);

    protected:
        virtual void setExternalResourcesRequiredBaseValue(SVGAnimatedPropertyTraits<bool>::PassType) = 0;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
