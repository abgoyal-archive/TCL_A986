

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFESpotLightElement.h"
#include "SVGSpotLightSource.h"

namespace WebCore {

SVGFESpotLightElement::SVGFESpotLightElement(const QualifiedName& tagName, Document* doc)
    : SVGFELightElement(tagName, doc)
{
}

SVGFESpotLightElement::~SVGFESpotLightElement()
{
}

PassRefPtr<LightSource> SVGFESpotLightElement::lightSource() const
{
    FloatPoint3D pos(x(), y(), z());

    // convert lookAt to a direction
    FloatPoint3D direction(pointsAtX() - pos.x(), 
                           pointsAtY() - pos.y(), 
                           pointsAtZ() - pos.z());

    direction.normalize();
    return SpotLightSource::create(pos, direction, specularExponent(), limitingConeAngle());
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
