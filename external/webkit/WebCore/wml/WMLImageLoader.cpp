

#include "config.h"

#if ENABLE(WML)
#include "WMLImageLoader.h"

#include "CachedImage.h"
#include "CSSHelper.h"
#include "HTMLNames.h"
#include "WMLImageElement.h"
#include "WMLNames.h"

namespace WebCore {

using namespace WMLNames;

WMLImageLoader::WMLImageLoader(WMLImageElement* element)
    : ImageLoader(element)
{
}

WMLImageLoader::~WMLImageLoader()
{
}

void WMLImageLoader::dispatchLoadEvent()
{
    // WML doesn't fire any events.
}

String WMLImageLoader::sourceURI(const AtomicString& attr) const
{
    return deprecatedParseURL(KURL(element()->baseURI(), attr).string());
}

void WMLImageLoader::notifyFinished(CachedResource* image)
{
    ImageLoader::notifyFinished(image);

    if (!image->errorOccurred())
        return;

    WMLImageElement* imageElement = static_cast<WMLImageElement*>(element());
    ASSERT(imageElement);

    // Loading both 'localsrc' and 'src' failed. Ignore this image.
    if (imageElement->useFallbackAttribute())
        return;

    if (!imageElement->hasAttribute(localsrcAttr) && !imageElement->hasAttribute(HTMLNames::srcAttr))
        return;

    imageElement->setUseFallbackAttribute(true);
    updateFromElementIgnoringPreviousError();
}

}

#endif
