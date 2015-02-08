

#include "config.h"

#if ENABLE(WML)
#include "WMLIntrinsicEvent.h"

#include "HTMLNames.h"
#include "WMLElementFactory.h"
#include "WMLNames.h"
#include "WMLTaskElement.h"

namespace WebCore {

using namespace WMLNames;

static PassRefPtr<WMLTaskElement> createTaskElement(Document* document)
{
    return static_pointer_cast<WMLTaskElement>(WMLElementFactory::createWMLElement(goTag, document, false));
}

WMLIntrinsicEvent::WMLIntrinsicEvent(Document* document, const String& targetURL)
    : m_taskElement(createTaskElement(document))
{
    m_taskElement->setAttribute(HTMLNames::hrefAttr, targetURL);
}

WMLIntrinsicEvent::WMLIntrinsicEvent(WMLTaskElement* taskElement)
    : m_taskElement(taskElement)
{
}

}

#endif
