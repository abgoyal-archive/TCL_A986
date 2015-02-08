

#include "config.h"

#if ENABLE(SVG)
#include "SVGImageLoader.h"

#include "Event.h"
#include "EventNames.h"
#include "SVGImageElement.h"
#include "RenderImage.h"

namespace WebCore {

SVGImageLoader::SVGImageLoader(SVGImageElement* node)
    : ImageLoader(node)
{
}

SVGImageLoader::~SVGImageLoader()
{
}

void SVGImageLoader::dispatchLoadEvent()
{
    if (image()->errorOccurred())
        element()->dispatchEvent(Event::create(eventNames().errorEvent, false, false));
    else {
        SVGImageElement* imageElement = static_cast<SVGImageElement*>(element());
        if (imageElement->externalResourcesRequiredBaseValue())
            imageElement->sendSVGLoadEventIfPossible(true);
    }
}

String SVGImageLoader::sourceURI(const AtomicString& attr) const
{
    return deprecatedParseURL(KURL(element()->baseURI(), attr).string());
}

}

#endif // ENABLE(SVG)
