

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEDistantLightElement.h"
#include "SVGDistantLightSource.h"

namespace WebCore {

SVGFEDistantLightElement::SVGFEDistantLightElement(const QualifiedName& tagName, Document* doc)
    : SVGFELightElement(tagName, doc)
{
}

SVGFEDistantLightElement::~SVGFEDistantLightElement()
{
}

PassRefPtr<LightSource> SVGFEDistantLightElement::lightSource() const
{
    return DistantLightSource::create(azimuth(), elevation());
}

}

#endif // ENABLE(SVG)
