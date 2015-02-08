

#include "config.h"
#include "WebElement.h"

#include "Element.h"
#include <wtf/PassRefPtr.h>

using namespace WebCore;

namespace WebKit {

WebElement::WebElement(const WTF::PassRefPtr<WebCore::Element>& elem)
    : WebNode(elem.releaseRef())
{
}

WebElement& WebElement::operator=(const WTF::PassRefPtr<WebCore::Element>& elem)
{
    WebNode::assign(elem.releaseRef());
    return *this;
}

WebElement::operator WTF::PassRefPtr<Element>() const
{
    return PassRefPtr<Element>(static_cast<Element*>(m_private));
}

WebString WebElement::tagName() const
{
    return constUnwrap<Element>()->tagName();
}

bool WebElement::hasTagName(const WebString& tagName) const
{
    return equalIgnoringCase(constUnwrap<Element>()->tagName(),
                             tagName.operator WebCore::String());
}

bool WebElement::hasAttribute(const WebString& attrName) const
{
    return constUnwrap<Element>()->hasAttribute(attrName);
}

WebString WebElement::getAttribute(const WebString& attrName) const
{
    return constUnwrap<Element>()->getAttribute(attrName);
}

bool WebElement::setAttribute(const WebString& attrName, const WebString& attrValue)
{
    ExceptionCode exceptionCode = 0;
    unwrap<Element>()->setAttribute(attrName, attrValue, exceptionCode);
    return !exceptionCode;
}

WebString WebElement::innerText() const
{
    return constUnwrap<Element>()->innerText();
}

} // namespace WebKit

