

#include "config.h"

#if ENABLE(WML)
#include "WMLDocument.h"

#include "BackForwardList.h"
#include "Frame.h"
#include "Page.h"
#include "Tokenizer.h"
#include "WMLCardElement.h"
#include "WMLErrorHandling.h"
#include "WMLPageState.h"
#include "WMLTemplateElement.h"

namespace WebCore {

WMLDocument::WMLDocument(Frame* frame)
    : Document(frame, false, false) 
    , m_activeCard(0)
{
    clearXMLVersion();
}

WMLDocument::~WMLDocument()
{
}

void WMLDocument::finishedParsing()
{
    if (Tokenizer* tokenizer = this->tokenizer()) {
        if (!tokenizer->wellFormed()) {
            Document::finishedParsing();
            return;
        }
    }

    bool hasAccess = initialize(true);
    Document::finishedParsing();

    if (!hasAccess) {
        m_activeCard = 0;

        WMLPageState* wmlPageState = wmlPageStateForDocument(this);
        if (!wmlPageState)
            return;

        Page* page = wmlPageState->page();
        if (!page)
            return;

        BackForwardList* list = page->backForwardList();
        if (!list)
            return;

        HistoryItem* item = list->backItem();
        if (!item)
            return;

        page->goToItem(item, FrameLoadTypeBackWMLDeckNotAccessible);
        return;
    }

    if (m_activeCard)
        m_activeCard->handleIntrinsicEventIfNeeded();
}

bool WMLDocument::initialize(bool aboutToFinishParsing)
{
    WMLPageState* wmlPageState = wmlPageStateForDocument(this);
    if (!wmlPageState || !wmlPageState->canAccessDeck())
        return false;

    // Remember that we'e successfully entered the deck
    wmlPageState->resetAccessControlData();

    // Notify the existance of templates to all cards of the current deck
    WMLTemplateElement::registerTemplatesInDocument(this);

    // Set destination card
    m_activeCard = WMLCardElement::determineActiveCard(this);
    if (!m_activeCard) {
        reportWMLError(this, WMLErrorNoCardInDocument);
        return true;
    }

    // Handle deck-level task overrides
    m_activeCard->handleDeckLevelTaskOverridesIfNeeded();

    // Handle card-level intrinsic event
    if (!aboutToFinishParsing)
        m_activeCard->handleIntrinsicEventIfNeeded();

    return true;
}

WMLPageState* wmlPageStateForDocument(Document* doc)
{
    ASSERT(doc);

    Page* page = doc->page();
    ASSERT(page);

    return page->wmlPageState();
}

}

#endif
