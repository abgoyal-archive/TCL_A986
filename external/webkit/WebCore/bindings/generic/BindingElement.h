

#ifndef BindingElement_h
#define BindingElement_h

#include "Attr.h"
#include "BindingSecurity.h"
#include "Element.h"
#include "ExceptionCode.h"

#include <wtf/RefPtr.h>

namespace WebCore {

template <class Binding>
class BindingElement {
public:
    static void setAttribute(State<Binding>*, Element*, const AtomicString&, const AtomicString&, ExceptionCode&);
    static RefPtr<Attr> setAttributeNode(State<Binding>*, Element*, Attr*, ExceptionCode&);
    static void setAttributeNS(State<Binding>*, Element*, const AtomicString&, const AtomicString&, const AtomicString&, ExceptionCode&);
    static RefPtr<Attr> setAttributeNodeNS(State<Binding>*, Element*, Attr*, ExceptionCode&);
};

// Implementations of templated methods must be in this file.

template <class Binding>
void BindingElement<Binding>::setAttribute(State<Binding>* state, Element* element, const AtomicString& name, const AtomicString& value, ExceptionCode& ec)
{
    ASSERT(element);

    if (!BindingSecurity<Binding>::allowSettingSrcToJavascriptURL(state, element, name, value))
        return;

    element->setAttribute(name, value, ec);
}

template <class Binding>
RefPtr<Attr> BindingElement<Binding>::setAttributeNode(State<Binding>* state, Element* element, Attr* newAttr, ExceptionCode& ec)
{
    ASSERT(element);
    ASSERT(newAttr);

    if (!BindingSecurity<Binding>::allowSettingSrcToJavascriptURL(state, element, newAttr->name(), newAttr->value()))
        return 0;

    return element->setAttributeNode(newAttr, ec);
}

template <class Binding>
void BindingElement<Binding>::setAttributeNS(State<Binding>* state, Element* element, const AtomicString& namespaceURI, const AtomicString& qualifiedName, const AtomicString& value, ExceptionCode& ec)
{
    ASSERT(element);

    if (!BindingSecurity<Binding>::allowSettingSrcToJavascriptURL(state, element, qualifiedName, value))
        return;

    element->setAttributeNS(namespaceURI, qualifiedName, value, ec);
}

template <class Binding>
RefPtr<Attr> BindingElement<Binding>::setAttributeNodeNS(State<Binding>* state, Element* element, Attr* newAttr, ExceptionCode& ec)
{
    ASSERT(element);
    ASSERT(newAttr);

    if (!BindingSecurity<Binding>::allowSettingSrcToJavascriptURL(state, element, newAttr->name(), newAttr->value()))
        return 0;

    return element->setAttributeNodeNS(newAttr, ec);
}

} // namespace WebCore

#endif // BindingElement_h
