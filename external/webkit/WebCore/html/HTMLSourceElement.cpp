

#include "config.h"

#if ENABLE(VIDEO)
#include "HTMLSourceElement.h"

#include "Event.h"
#include "EventNames.h"
#include "HTMLDocument.h"
#include "HTMLMediaElement.h"
#include "HTMLNames.h"

using namespace std;

namespace WebCore {

using namespace HTMLNames;

HTMLSourceElement::HTMLSourceElement(const QualifiedName& tagName, Document* doc)
    : HTMLElement(tagName, doc)
    , m_errorEventTimer(this, &HTMLSourceElement::errorEventTimerFired)
{
    ASSERT(hasTagName(sourceTag));
}

HTMLSourceElement::~HTMLSourceElement()
{
}

void HTMLSourceElement::insertedIntoDocument()
{
    HTMLElement::insertedIntoDocument();
    if (parentNode() && (parentNode()->hasTagName(audioTag) ||  parentNode()->hasTagName(videoTag))) {
        HTMLMediaElement* media = static_cast<HTMLMediaElement*>(parentNode());
        if (media->networkState() == HTMLMediaElement::NETWORK_EMPTY)
            media->scheduleLoad();
    }
}

KURL HTMLSourceElement::src() const
{
    return document()->completeURL(getAttribute(srcAttr));
}

void HTMLSourceElement::setSrc(const String& url)
{
    setAttribute(srcAttr, url);
}

String HTMLSourceElement::media() const
{
    return getAttribute(mediaAttr);
}

void HTMLSourceElement::setMedia(const String& media)
{
    setAttribute(mediaAttr, media);
}

String HTMLSourceElement::type() const
{
    return getAttribute(typeAttr);
}

void HTMLSourceElement::setType(const String& type)
{
    setAttribute(typeAttr, type);
}

void HTMLSourceElement::scheduleErrorEvent()
{
    if (m_errorEventTimer.isActive())
        return;

    m_errorEventTimer.startOneShot(0);
}

void HTMLSourceElement::cancelPendingErrorEvent()
{
    m_errorEventTimer.stop();
}

void HTMLSourceElement::errorEventTimerFired(Timer<HTMLSourceElement>*)
{
    dispatchEvent(Event::create(eventNames().errorEvent, false, true));
}

}
#endif
