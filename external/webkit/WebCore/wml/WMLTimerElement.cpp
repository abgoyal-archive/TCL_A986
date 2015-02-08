

#include "config.h"

#if ENABLE(WML)
#include "WMLTimerElement.h"

#include "HTMLNames.h"
#include "MappedAttribute.h"
#include "WMLCardElement.h"
#include "WMLDocument.h"
#include "WMLNames.h"
#include "WMLPageState.h"
#include "WMLTemplateElement.h"
#include "WMLVariables.h"

namespace WebCore {

using namespace WMLNames;

WMLTimerElement::WMLTimerElement(const QualifiedName& tagName, Document* doc)
    : WMLElement(tagName, doc)
    , m_timer(this, &WMLTimerElement::timerFired)
{
}

void WMLTimerElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == HTMLNames::nameAttr)
        m_name = parseValueForbiddingVariableReferences(attr->value());
    else
        WMLElement::parseMappedAttribute(attr);
}

void WMLTimerElement::insertedIntoDocument()
{
    WMLElement::insertedIntoDocument();

    // If the value of timeout is not a positive integer, ignore it
    if (value().toInt() <= 0)
        return;

    Node* parent = parentNode();
    if (!parent || !parent->isWMLElement())
        return;

    if (parent->hasTagName(cardTag)) {
        m_card = static_cast<WMLCardElement*>(parent);
        m_card->setIntrinsicEventTimer(this);
    }
}

void WMLTimerElement::removedFromDocument()
{
    Node* parent = parentNode();
    if (parent && parent->isWMLElement() && parent->hasTagName(cardTag)) {
        m_card->setIntrinsicEventTimer(0);
        m_card = 0;
    }

    WMLElement::removedFromDocument();
}

void WMLTimerElement::timerFired(Timer<WMLTimerElement>*)
{
    if (!m_card)
        return;

    WMLPageState* pageState = wmlPageStateForDocument(document());
    if (!pageState)
        return;

    String value = this->value();

    // When the timer expires, set the name varialbe of timer to '0'
    if (!m_name.isEmpty()) {
        value = "0";
        pageState->storeVariable(m_name, value);
    }

    WMLIntrinsicEventType eventType = WMLIntrinsicEventOnTimer;
    WMLIntrinsicEventHandler* eventHandler = m_card->eventHandler();

    bool hasIntrinsicEvent = false;
    if (eventHandler && eventHandler->hasIntrinsicEvent(eventType))
        hasIntrinsicEvent = true;
    else if (m_card->templateElement()) {
        eventHandler = m_card->templateElement()->eventHandler();
        if (eventHandler && eventHandler->hasIntrinsicEvent(eventType))
            hasIntrinsicEvent = true;
    }

    if (hasIntrinsicEvent)
        eventHandler->triggerIntrinsicEvent(eventType);
}

void WMLTimerElement::start(int interval)
{
    if (!m_card || m_timer.isActive())
        return;

    if (interval <= 0 && !m_name.isEmpty()) {
        if (WMLPageState* pageState = wmlPageStateForDocument(document()))
            interval = pageState->getVariable(m_name).toInt();
    }

    if (interval <= 0)
        interval = value().toInt();

    if (interval > 0)
        m_timer.startOneShot(interval / 10.0f);
}

void WMLTimerElement::stop()
{
    if (m_timer.isActive())
        m_timer.stop();
}

void WMLTimerElement::storeIntervalToPageState()
{
    if (!m_timer.isActive())
        return;

    int interval = static_cast<int>(m_timer.nextFireInterval()) * 10;

    if (WMLPageState* pageState = wmlPageStateForDocument(document()))
        pageState->storeVariable(m_name, String::number(interval));
}

String WMLTimerElement::value() const
{
    return parseValueSubstitutingVariableReferences(getAttribute(HTMLNames::valueAttr));
}

}

#endif
