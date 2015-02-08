

#include "config.h"

#if ENABLE(WML)
#include "WMLPrevElement.h"

#include "Page.h"
#include "WMLCardElement.h"
#include "WMLDocument.h"
#include "WMLPageState.h"
#include "WMLTimerElement.h"

namespace WebCore {

WMLPrevElement::WMLPrevElement(const QualifiedName& tagName, Document* doc)
    : WMLTaskElement(tagName, doc)
{
}

WMLPrevElement::~WMLPrevElement()
{
}

void WMLPrevElement::executeTask()
{
    ASSERT(document()->isWMLDocument());
    WMLDocument* document = static_cast<WMLDocument*>(this->document());

    WMLPageState* pageState = wmlPageStateForDocument(document);
    if (!pageState)
        return;

    WMLCardElement* card = document->activeCard();
    if (!card)
        return;

    storeVariableState(pageState);

    // Stop the timer of the current card if it is active
    if (WMLTimerElement* eventTimer = card->eventTimer())
        eventTimer->stop();

    pageState->page()->goBack();
}

}

#endif
