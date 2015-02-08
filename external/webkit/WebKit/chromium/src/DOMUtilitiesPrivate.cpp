

#include "config.h"
#include "DOMUtilitiesPrivate.h"

#include "Element.h"
#include "HTMLInputElement.h"
#include "HTMLLinkElement.h"
#include "HTMLMetaElement.h"
#include "HTMLNames.h"
#include "HTMLOptionElement.h"
#include "Node.h"

#include "WebInputElement.h"

using namespace WebCore;

namespace {

template <class HTMLNodeType>
HTMLNodeType* toHTMLElement(Node* node, const QualifiedName& name)
{
    if (node->isHTMLElement()
        && static_cast<HTMLElement*>(node)->hasTagName(name)) {
        return static_cast<HTMLNodeType*>(node);
    }
    return 0;
}

} // namespace

namespace WebKit {

HTMLInputElement* toHTMLInputElement(Node* node)
{
    return toHTMLElement<HTMLInputElement>(node, HTMLNames::inputTag);
}

HTMLLinkElement* toHTMLLinkElement(Node* node)
{
    return toHTMLElement<HTMLLinkElement>(node, HTMLNames::linkTag);
}

HTMLMetaElement* toHTMLMetaElement(Node* node)
{
    return toHTMLElement<HTMLMetaElement>(node, HTMLNames::metaTag);
}

HTMLOptionElement* toHTMLOptionElement(Node* node)
{
    return toHTMLElement<HTMLOptionElement>(node, HTMLNames::optionTag);
}

String nameOfInputElement(HTMLInputElement* element)
{
    return WebInputElement(element).nameForAutofill();
}

bool elementHasLegalLinkAttribute(const Element* element,
                                  const QualifiedName& attrName)
{
    if (attrName == HTMLNames::srcAttr) {
        // Check src attribute.
        if (element->hasTagName(HTMLNames::imgTag)
            || element->hasTagName(HTMLNames::scriptTag)
            || element->hasTagName(HTMLNames::iframeTag)
            || element->hasTagName(HTMLNames::frameTag))
            return true;
        if (element->hasTagName(HTMLNames::inputTag)) {
            const HTMLInputElement* input =
            static_cast<const HTMLInputElement*>(element);
            if (input->inputType() == HTMLInputElement::IMAGE)
                return true;
        }
    } else if (attrName == HTMLNames::hrefAttr) {
        // Check href attribute.
        if (element->hasTagName(HTMLNames::linkTag)
            || element->hasTagName(HTMLNames::aTag)
            || element->hasTagName(HTMLNames::areaTag))
            return true;
    } else if (attrName == HTMLNames::actionAttr) {
        if (element->hasTagName(HTMLNames::formTag))
            return true;
    } else if (attrName == HTMLNames::backgroundAttr) {
        if (element->hasTagName(HTMLNames::bodyTag)
            || element->hasTagName(HTMLNames::tableTag)
            || element->hasTagName(HTMLNames::trTag)
            || element->hasTagName(HTMLNames::tdTag))
            return true;
    } else if (attrName == HTMLNames::citeAttr) {
        if (element->hasTagName(HTMLNames::blockquoteTag)
            || element->hasTagName(HTMLNames::qTag)
            || element->hasTagName(HTMLNames::delTag)
            || element->hasTagName(HTMLNames::insTag))
            return true;
    } else if (attrName == HTMLNames::classidAttr
               || attrName == HTMLNames::dataAttr) {
        if (element->hasTagName(HTMLNames::objectTag))
            return true;
    } else if (attrName == HTMLNames::codebaseAttr) {
        if (element->hasTagName(HTMLNames::objectTag)
            || element->hasTagName(HTMLNames::appletTag))
            return true;
    }
    return false;
}

} // namespace WebKit
