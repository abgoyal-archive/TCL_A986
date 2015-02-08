

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEPointLightElement.h"
#include "SVGPointLightSource.h"

namespace WebCore {

SVGFEPointLightElement::SVGFEPointLightElement(const QualifiedName& tagName, Document* doc)
    : SVGFELightElement(tagName, doc)
{
}

SVGFEPointLightElement::~SVGFEPointLightElement()
{
}

PassRefPtr<LightSource> SVGFEPointLightElement::lightSource() const
{
    FloatPoint3D pos(x(), y(), z());
    return PointLightSource::create(pos);
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
