

#include "config.h"
#include "WebInputElement.h"

#include "HTMLInputElement.h"
#include "HTMLNames.h"
#include "WebString.h"
#include <wtf/PassRefPtr.h>

using namespace WebCore;

namespace WebKit {

WebInputElement::WebInputElement(const WTF::PassRefPtr<HTMLInputElement>& elem)
    : WebElement(elem.releaseRef())
{
}

WebInputElement& WebInputElement::operator=(const WTF::PassRefPtr<HTMLInputElement>& elem)
{
    WebNode::assign(elem.releaseRef());
    return *this;
}

WebInputElement::operator WTF::PassRefPtr<HTMLInputElement>() const
{
    return PassRefPtr<HTMLInputElement>(static_cast<HTMLInputElement*>(m_private));
}

bool WebInputElement::autoComplete() const
{
    return constUnwrap<HTMLInputElement>()->autoComplete();
}

bool WebInputElement::isEnabledFormControl() const
{
    return constUnwrap<HTMLInputElement>()->isEnabledFormControl();
}

WebInputElement::InputType WebInputElement::inputType() const
{
    return static_cast<InputType>(constUnwrap<HTMLInputElement>()->inputType());
}

WebString WebInputElement::formControlType() const
{
    return constUnwrap<HTMLInputElement>()->formControlType();
}

bool WebInputElement::isActivatedSubmit() const
{
    return constUnwrap<HTMLInputElement>()->isActivatedSubmit();
}

void WebInputElement::setActivatedSubmit(bool activated)
{
    unwrap<HTMLInputElement>()->setActivatedSubmit(activated);
}

void WebInputElement::setValue(const WebString& value)
{
    unwrap<HTMLInputElement>()->setValue(value);
}

WebString WebInputElement::value() const
{
    return constUnwrap<HTMLInputElement>()->value();
}

void WebInputElement::setAutofilled(bool autoFilled)
{
    unwrap<HTMLInputElement>()->setAutofilled(autoFilled);
}

void WebInputElement::dispatchFormControlChangeEvent()
{
    unwrap<HTMLInputElement>()->dispatchFormControlChangeEvent();
}

void WebInputElement::setSelectionRange(int start, int end)
{
    unwrap<HTMLInputElement>()->setSelectionRange(start, end);
}
    
WebString WebInputElement::name() const
{
    return constUnwrap<HTMLInputElement>()->name();
}
    
WebString WebInputElement::nameForAutofill() const
{
    String name = constUnwrap<HTMLInputElement>()->name();
    String trimmedName = name.stripWhiteSpace();
    if (!trimmedName.isEmpty())
        return trimmedName;
    name = constUnwrap<HTMLInputElement>()->getAttribute(HTMLNames::idAttr);
    trimmedName = name.stripWhiteSpace();
    if (!trimmedName.isEmpty())
        return trimmedName;
    return String();
}

} // namespace WebKit
