

#ifndef SVGFETurbulenceElement_h
#define SVGFETurbulenceElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFETurbulence.h"
#include "SVGFilterPrimitiveStandardAttributes.h"

namespace WebCore {

    extern char SVGBaseFrequencyXIdentifier[];
    extern char SVGBaseFrequencyYIdentifier[];

    enum SVGStitchOptions {
        SVG_STITCHTYPE_UNKNOWN  = 0,
        SVG_STITCHTYPE_STITCH   = 1,
        SVG_STITCHTYPE_NOSTITCH = 2
    };

    class SVGFETurbulenceElement : public SVGFilterPrimitiveStandardAttributes {
    public:
        SVGFETurbulenceElement(const QualifiedName&, Document*);
        virtual ~SVGFETurbulenceElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual bool build(SVGResourceFilter*);

    private:
        DECLARE_ANIMATED_PROPERTY_MULTIPLE_WRAPPERS(SVGFETurbulenceElement, SVGNames::baseFrequencyAttr, SVGBaseFrequencyXIdentifier, float, BaseFrequencyX, baseFrequencyX)
        DECLARE_ANIMATED_PROPERTY_MULTIPLE_WRAPPERS(SVGFETurbulenceElement, SVGNames::baseFrequencyAttr, SVGBaseFrequencyYIdentifier, float, BaseFrequencyY, baseFrequencyY)
        DECLARE_ANIMATED_PROPERTY(SVGFETurbulenceElement, SVGNames::numOctavesAttr, long, NumOctaves, numOctaves)
        DECLARE_ANIMATED_PROPERTY(SVGFETurbulenceElement, SVGNames::seedAttr, float, Seed, seed)
        DECLARE_ANIMATED_PROPERTY(SVGFETurbulenceElement, SVGNames::stitchTilesAttr, int, StitchTiles, stitchTiles)
        DECLARE_ANIMATED_PROPERTY(SVGFETurbulenceElement, SVGNames::typeAttr, int, Type, type)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
