

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFETurbulenceElement.h"

#include "MappedAttribute.h"
#include "SVGParserUtilities.h"
#include "SVGResourceFilter.h"

namespace WebCore {

char SVGBaseFrequencyXIdentifier[] = "SVGBaseFrequencyX";
char SVGBaseFrequencyYIdentifier[] = "SVGBaseFrequencyY";

SVGFETurbulenceElement::SVGFETurbulenceElement(const QualifiedName& tagName, Document* doc)
    : SVGFilterPrimitiveStandardAttributes(tagName, doc)
    , m_numOctaves(1)
    , m_stitchTiles(SVG_STITCHTYPE_NOSTITCH)
    , m_type(FETURBULENCE_TYPE_TURBULENCE)
{
}

SVGFETurbulenceElement::~SVGFETurbulenceElement()
{
}

void SVGFETurbulenceElement::parseMappedAttribute(MappedAttribute* attr)
{
    const String& value = attr->value();
    if (attr->name() == SVGNames::typeAttr) {
        if (value == "fractalNoise")
            setTypeBaseValue(FETURBULENCE_TYPE_FRACTALNOISE);
        else if (value == "turbulence")
            setTypeBaseValue(FETURBULENCE_TYPE_TURBULENCE);
    } else if (attr->name() == SVGNames::stitchTilesAttr) {
        if (value == "stitch")
            setStitchTilesBaseValue(SVG_STITCHTYPE_STITCH);
        else if (value == "nostitch")
            setStitchTilesBaseValue(SVG_STITCHTYPE_NOSTITCH);
    } else if (attr->name() == SVGNames::baseFrequencyAttr) {
        float x, y;
        if (parseNumberOptionalNumber(value, x, y)) {
            setBaseFrequencyXBaseValue(x);
            setBaseFrequencyYBaseValue(y);
        }
    } else if (attr->name() == SVGNames::seedAttr)
        setSeedBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::numOctavesAttr)
        setNumOctavesBaseValue(value.toUIntStrict());
    else
        SVGFilterPrimitiveStandardAttributes::parseMappedAttribute(attr);
}

void SVGFETurbulenceElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGFilterPrimitiveStandardAttributes::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizeType();
        synchronizeStitchTiles();
        synchronizeBaseFrequencyX();
        synchronizeBaseFrequencyY();
        synchronizeSeed();
        synchronizeNumOctaves();
        return;
    }

    if (attrName == SVGNames::typeAttr)
        synchronizeType();
    else if (attrName == SVGNames::stitchTilesAttr)
        synchronizeStitchTiles();
    else if (attrName == SVGNames::baseFrequencyAttr) {
        synchronizeBaseFrequencyX();
        synchronizeBaseFrequencyY();
    } else if (attrName == SVGNames::seedAttr)
        synchronizeSeed();
    else if (attrName == SVGNames::numOctavesAttr)
        synchronizeNumOctaves();
}

bool SVGFETurbulenceElement::build(SVGResourceFilter* filterResource)
{
    RefPtr<FilterEffect> effect = FETurbulence::create(static_cast<TurbulanceType>(type()), baseFrequencyX(), 
                                        baseFrequencyY(), numOctaves(), seed(), stitchTiles() == SVG_STITCHTYPE_STITCH);
    filterResource->addFilterEffect(this, effect.release());

    return true;
}

}

#endif // ENABLE(SVG)
