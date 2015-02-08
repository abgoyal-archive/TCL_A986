

#include "config.h"
#include "WebFormElement.h"

#include "HTMLFormControlElement.h"
#include "HTMLFormElement.h"
#include "HTMLInputElement.h"
#include "HTMLNames.h"
#include "WebString.h"
#include "WebURL.h"
#include <wtf/PassRefPtr.h>

using namespace WebCore;

namespace WebKit {

class WebFormPrivate : public HTMLFormElement {
};

WebFormElement::WebFormElement(const WTF::PassRefPtr<HTMLFormElement>& e)
    : WebElement(e.releaseRef())
{
}

WebFormElement& WebFormElement::operator=(const WTF::PassRefPtr<HTMLFormElement>& e)
{
    WebNode::assign(e.releaseRef());
    return *this;
}

WebFormElement::operator WTF::PassRefPtr<WebCore::HTMLFormElement>() const
{
    return PassRefPtr<HTMLFormElement>(static_cast<HTMLFormElement*>(m_private));
}

bool WebFormElement::autoComplete() const
{
    return constUnwrap<HTMLFormElement>()->autoComplete();
}

WebString WebFormElement::action() const
{
    return constUnwrap<HTMLFormElement>()->action();
}

WebString WebFormElement::name() const 
{
    return constUnwrap<HTMLFormElement>()->name();
}

WebString WebFormElement::method() const 
{
    return constUnwrap<HTMLFormElement>()->method();
}
    
void WebFormElement::submit()
{
    unwrap<HTMLFormElement>()->submit();
}

void WebFormElement::getNamedElements(const WebString& name,
                                      WebVector<WebNode>& result)
{
    Vector<RefPtr<Node> > tempVector;
    unwrap<HTMLFormElement>()->getNamedElements(name, tempVector);
    result.assign(tempVector);
}
    
void WebFormElement::getInputElements(WebVector<WebInputElement>& result) const
{
    const HTMLFormElement* form = constUnwrap<HTMLFormElement>();
    Vector<RefPtr<HTMLInputElement> > tempVector;
    for (size_t i = 0; i < form->formElements.size(); i++) {
        if (form->formElements[i]->hasLocalName(HTMLNames::inputTag))
            tempVector.append(static_cast<HTMLInputElement*>(
                form->formElements[i]));
    }
    result.assign(tempVector);
}

} // namespace WebKit
