

#include "config.h"
#include "HTMLLabelElement.h"

#include "Document.h"
#include "Event.h"
#include "EventNames.h"
#include "HTMLFormElement.h"
#include "HTMLNames.h"

namespace WebCore {

using namespace HTMLNames;

HTMLLabelElement::HTMLLabelElement(const QualifiedName& tagName, Document *doc)
    : HTMLElement(tagName, doc)
{
    ASSERT(hasTagName(labelTag));
}

HTMLLabelElement::~HTMLLabelElement()
{
}

bool HTMLLabelElement::isFocusable() const
{
    return false;
}

HTMLElement* HTMLLabelElement::correspondingControl()
{
    const AtomicString& controlId = getAttribute(forAttr);
    if (controlId.isNull()) {
        // Search children of the label element for a form element.
        Node* node = this;
        while ((node = node->traverseNextNode(this))) {
            if (node->isHTMLElement()) {
                HTMLElement* element = static_cast<HTMLElement*>(node);
                if (element->isFormControlElement())
                    return element;
            }
        }
        return 0;
    }
        
    // Only return HTML elements.
    Element* elt = document()->getElementById(controlId);
    if (elt && elt->isHTMLElement())
        return static_cast<HTMLElement*>(elt);
    return 0;
}

void HTMLLabelElement::setActive(bool down, bool pause)
{
    if (down == active())
        return;

    // Update our status first.
    HTMLElement::setActive(down, pause);

    // Also update our corresponding control.
    if (HTMLElement* element = correspondingControl())
        element->setActive(down, pause);
}

void HTMLLabelElement::setHovered(bool over)
{
    if (over == hovered())
        return;
        
    // Update our status first.
    HTMLElement::setHovered(over);

    // Also update our corresponding control.
    if (HTMLElement* element = correspondingControl())
        element->setHovered(over);
}

void HTMLLabelElement::defaultEventHandler(Event* evt)
{
    static bool processingClick = false;

    if (evt->type() == eventNames().clickEvent && !processingClick) {
        RefPtr<HTMLElement> control = correspondingControl();

        // If we can't find a control or if the control received the click
        // event, then there's no need for us to do anything.
        if (!control || (evt->target() && control->contains(evt->target()->toNode())))
            return;

        processingClick = true;

        // Click the corresponding control.
        control->dispatchSimulatedClick(evt);

        // If the control can be focused via the mouse, then do that too.
        if (control->isMouseFocusable())
            control->focus();

        processingClick = false;
        
        evt->setDefaultHandled();
    }
    
    HTMLElement::defaultEventHandler(evt);
}

void HTMLLabelElement::focus(bool)
{
    // to match other browsers, always restore previous selection
    if (HTMLElement* element = correspondingControl())
        element->focus();
}

void HTMLLabelElement::accessKeyAction(bool sendToAnyElement)
{
    if (HTMLElement* element = correspondingControl())
        element->accessKeyAction(sendToAnyElement);
}

String HTMLLabelElement::accessKey() const
{
    return getAttribute(accesskeyAttr);
}

void HTMLLabelElement::setAccessKey(const String &value)
{
    setAttribute(accesskeyAttr, value);
}

String HTMLLabelElement::htmlFor() const
{
    return getAttribute(forAttr);
}

void HTMLLabelElement::setHtmlFor(const String &value)
{
    setAttribute(forAttr, value);
}

} // namespace
