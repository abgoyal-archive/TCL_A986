

#include "config.h"

#if ENABLE(WML)
#include "WMLOnEventElement.h"

#include "HTMLNames.h"
#include "MappedAttribute.h"
#include "WMLErrorHandling.h"
#include "WMLEventHandlingElement.h"
#include "WMLIntrinsicEventHandler.h"
#include "WMLNames.h"
#include "WMLVariables.h"

namespace WebCore {

using namespace WMLNames;

WMLOnEventElement::WMLOnEventElement(const QualifiedName& tagName, Document* doc)
    : WMLElement(tagName, doc)
    , m_type(WMLIntrinsicEventUnknown)
{
}

void WMLOnEventElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == HTMLNames::typeAttr) {
        String parsedValue = parseValueForbiddingVariableReferences(attr->value());
        if (parsedValue.isEmpty())
            return;

        if (parsedValue == onenterforwardAttr)
            m_type = WMLIntrinsicEventOnEnterForward;
        else if (parsedValue == onenterbackwardAttr)
            m_type = WMLIntrinsicEventOnEnterBackward;
        else if (parsedValue == ontimerAttr)
            m_type = WMLIntrinsicEventOnTimer;
        else if (parsedValue == onpickAttr)
            m_type = WMLIntrinsicEventOnPick;
    } else
        WMLElement::parseMappedAttribute(attr);
}

static inline WMLEventHandlingElement* eventHandlingParent(Node* parent)
{
    if (!parent || !parent->isWMLElement())
        return 0;

    return toWMLEventHandlingElement(static_cast<WMLElement*>(parent));
}

void WMLOnEventElement::registerTask(WMLTaskElement* task)
{
    if (m_type == WMLIntrinsicEventUnknown)
        return;

    // Register intrinsic event to the event handler of the owner of onevent element 
    WMLEventHandlingElement* eventHandlingElement = eventHandlingParent(parentNode());
    if (!eventHandlingElement)
        return;

    eventHandlingElement->createEventHandlerIfNeeded();

    RefPtr<WMLIntrinsicEvent> event = WMLIntrinsicEvent::createWithTask(task);
    if (!eventHandlingElement->eventHandler()->registerIntrinsicEvent(m_type, event))
        reportWMLError(document(), WMLErrorConflictingEventBinding);
}

void WMLOnEventElement::deregisterTask(WMLTaskElement*)
{
    WMLEventHandlingElement* eventHandlingElement = eventHandlingParent(parentNode());
    if (!eventHandlingElement)
        return;

    eventHandlingElement->eventHandler()->deregisterIntrinsicEvent(m_type);
}

}

#endif
