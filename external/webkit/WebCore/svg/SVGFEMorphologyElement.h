

#ifndef SVGFEMorphologyElement_h
#define SVGFEMorphologyElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEMorphology.h"
#include "SVGFilterPrimitiveStandardAttributes.h"

namespace WebCore {

    extern char SVGRadiusXAttrIdentifier[];
    extern char SVGRadiusYAttrIdentifier[];

    class SVGFEMorphologyElement : public SVGFilterPrimitiveStandardAttributes {
    public:
        SVGFEMorphologyElement(const QualifiedName&, Document*);
        virtual ~SVGFEMorphologyElement();

        void setRadius(float radiusX, float radiusY);

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual bool build(SVGResourceFilter*);

    private:
        DECLARE_ANIMATED_PROPERTY(SVGFEMorphologyElement, SVGNames::inAttr, String, In1, in1)
        DECLARE_ANIMATED_PROPERTY(SVGFEMorphologyElement, SVGNames::operatorAttr, int, _operator, _operator)
        DECLARE_ANIMATED_PROPERTY_MULTIPLE_WRAPPERS(SVGFEMorphologyElement, SVGNames::radiusAttr, SVGRadiusXAttrIdentifier, float, RadiusX, radiusX)
        DECLARE_ANIMATED_PROPERTY_MULTIPLE_WRAPPERS(SVGFEMorphologyElement, SVGNames::radiusAttr, SVGRadiusYAttrIdentifier, float, RadiusY, radiusY)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
