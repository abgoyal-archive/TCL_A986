

#include "config.h"

#if ENABLE(WML)
#include "WMLAnchorElement.h"

#include "EventNames.h"
#include "KeyboardEvent.h"
#include "WMLTaskElement.h"
#include "HTMLNames.h"

namespace WebCore {

WMLAnchorElement::WMLAnchorElement(const QualifiedName& tagName, Document* doc)
    : WMLAElement(tagName, doc)
    , m_task(0)
{
    // Calling setIsLink(true), and returning a non-null value on CSSStyleSelectors' linkAttribute
    // method, makes it possible to 'appear as link' (just like <a href="..">) without the need to
    // actually set the href value to an empty value in the DOM tree.
    setIsLink(true);
}

WMLAnchorElement::~WMLAnchorElement()
{
}

void WMLAnchorElement::defaultEventHandler(Event* event)
{
    bool shouldHandle = false;

    if (event->type() == eventNames().clickEvent)
        shouldHandle = true;
    else if (event->type() == eventNames().keydownEvent && event->isKeyboardEvent() && focused())
        shouldHandle = static_cast<KeyboardEvent*>(event)->keyIdentifier() == "Enter";

    if (shouldHandle && m_task) {
        m_task->executeTask();
        event->setDefaultHandled();
        return;
    }

    // Skip WMLAElement::defaultEventHandler, we don't own a href attribute, that needs to be handled.
    WMLElement::defaultEventHandler(event); 
}

void WMLAnchorElement::registerTask(WMLTaskElement* task)
{
    ASSERT(!m_task);
    m_task = task;
}

void WMLAnchorElement::deregisterTask(WMLTaskElement* task)
{
    ASSERT_UNUSED(task, m_task == task);
    m_task = 0;
}

}

#endif
