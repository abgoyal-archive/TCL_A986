

#include "config.h"
#include "HTMLImageLoader.h"

#include "CSSHelper.h"
#include "CachedImage.h"
#include "Element.h"
#include "Event.h"
#include "EventNames.h"
#include "HTMLNames.h"
#include "HTMLObjectElement.h"

namespace WebCore {

HTMLImageLoader::HTMLImageLoader(Element* node)
    : ImageLoader(node)
{
}

HTMLImageLoader::~HTMLImageLoader()
{
}

void HTMLImageLoader::dispatchLoadEvent()
{
    bool errorOccurred = image()->errorOccurred();
    if (!errorOccurred && image()->httpStatusCodeErrorOccurred())
        errorOccurred = element()->hasTagName(HTMLNames::objectTag); // An <object> considers a 404 to be an error and should fire onerror.
    element()->dispatchEvent(Event::create(errorOccurred ? eventNames().errorEvent : eventNames().loadEvent, false, false));
}

String HTMLImageLoader::sourceURI(const AtomicString& attr) const
{
    return deprecatedParseURL(attr);
}

void HTMLImageLoader::notifyFinished(CachedResource*)
{    
    CachedImage* cachedImage = image();

    Element* elem = element();
    ImageLoader::notifyFinished(cachedImage);

    if ((cachedImage->errorOccurred() || cachedImage->httpStatusCodeErrorOccurred()) && elem->hasTagName(HTMLNames::objectTag))
        static_cast<HTMLObjectElement*>(elem)->renderFallbackContent();
}

}
