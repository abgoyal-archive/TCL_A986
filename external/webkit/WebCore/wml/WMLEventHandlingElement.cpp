

#include "config.h"

#if ENABLE(WML)
#include "WMLEventHandlingElement.h"

#include "WMLCardElement.h"
#include "WMLDoElement.h"
#include "WMLIntrinsicEventHandler.h"
#include "WMLOptionElement.h"
#include "WMLTaskElement.h"
#include "WMLTemplateElement.h"
#include "WMLNames.h"

namespace WebCore {

using namespace WMLNames;

WMLEventHandlingElement::WMLEventHandlingElement()
{
}

WMLEventHandlingElement::~WMLEventHandlingElement()
{
}

void WMLEventHandlingElement::createEventHandlerIfNeeded()
{
    if (!m_eventHandler)
        m_eventHandler.set(new WMLIntrinsicEventHandler);
}

void WMLEventHandlingElement::registerDoElement(WMLDoElement* doElement, Document* document)
{
    Vector<WMLDoElement*>::iterator it = m_doElements.begin();
    Vector<WMLDoElement*>::iterator end = m_doElements.end();

    for (; it != end; ++it) {
        if ((*it)->name() == doElement->name()) {
            reportWMLError(document, WMLErrorDuplicatedDoElement);
            return;
        }
    }

    ASSERT(m_doElements.find(doElement) == WTF::notFound);
    m_doElements.append(doElement);
    doElement->setActive(true);
}

void WMLEventHandlingElement::deregisterDoElement(WMLDoElement* doElement)
{
    doElement->setActive(false);

    size_t position = m_doElements.find(doElement);
    if (position == WTF::notFound)
        return;

    m_doElements.remove(position);
}

WMLEventHandlingElement* toWMLEventHandlingElement(WMLElement* element)
{
    if (!element->isWMLElement())
        return 0;

    if (element->hasTagName(cardTag))
        return static_cast<WMLCardElement*>(element);
    else if (element->hasTagName(optionTag))
        return static_cast<WMLOptionElement*>(element);
    else if (element->hasTagName(templateTag))
        return static_cast<WMLTemplateElement*>(element);

    return 0;
}

}

#endif
