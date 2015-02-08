

#ifndef DOMUtilitiesPrivate_h
#define DOMUtilitiesPrivate_h

namespace WebCore {
class Element;
class HTMLInputElement;
class HTMLLinkElement;
class HTMLMetaElement;
class HTMLOptionElement;
class Node;
class QualifiedName;
class String;
}

// This file is an aggregate of useful WebCore operations.
namespace WebKit {

// If node is an HTML node with a tag name of name it is casted and returned.
// If node is not an HTML node or the tag name is not name, 0 is returned.
WebCore::HTMLInputElement* toHTMLInputElement(WebCore::Node*);
WebCore::HTMLLinkElement* toHTMLLinkElement(WebCore::Node*);
WebCore::HTMLMetaElement* toHTMLMetaElement(WebCore::Node*);
WebCore::HTMLOptionElement* toHTMLOptionElement(WebCore::Node*);

// FIXME: Deprecate. Use WebInputElement::nameForAutofill instead.
WebCore::String nameOfInputElement(WebCore::HTMLInputElement*);

// For img, script, iframe, frame element, when attribute name is src,
// for link, a, area element, when attribute name is href,
// for form element, when attribute name is action,
// for input, type=image, when attribute name is src,
// for body, table, tr, td, when attribute name is background,
// for blockquote, q, del, ins, when attribute name is cite,
// we can consider the attribute value has legal link.
bool elementHasLegalLinkAttribute(const WebCore::Element* element,
                                  const WebCore::QualifiedName& attrName);

} // namespace WebKit

#endif
