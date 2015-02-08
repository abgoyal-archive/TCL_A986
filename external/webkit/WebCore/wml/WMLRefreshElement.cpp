

#include "config.h"

#if ENABLE(WML)
#include "WMLRefreshElement.h"

#include "Frame.h"
#include "FrameLoader.h"
#include "WMLCardElement.h"
#include "WMLDocument.h"
#include "WMLPageState.h"
#include "WMLTimerElement.h"

namespace WebCore {

WMLRefreshElement::WMLRefreshElement(const QualifiedName& tagName, Document* doc)
    : WMLTaskElement(tagName, doc)
{
}

WMLRefreshElement::~WMLRefreshElement()
{
}

void WMLRefreshElement::executeTask()
{
    ASSERT(document()->isWMLDocument());
    WMLDocument* document = static_cast<WMLDocument*>(this->document());

    WMLPageState* pageState = wmlPageStateForDocument(document);
    if (!pageState)
        return;

    WMLCardElement* card = document->activeCard();
    if (!card)
        return;

    // Before perform refresh task, we store the current timeout
    // value in the page state and then stop the timer
    WMLTimerElement* timer = card->eventTimer();
    if (timer) {
        timer->storeIntervalToPageState();
        timer->stop();
    }

    storeVariableState(pageState);

    // Redisplay curremt card with current variable state
    if (Frame* frame = document->frame()) {
        if (FrameLoader* loader = frame->loader())
            loader->reload();
    }

    // After refreshing task, resume the timer if it exsits 
    if (timer)
        timer->start();
}

}

#endif
