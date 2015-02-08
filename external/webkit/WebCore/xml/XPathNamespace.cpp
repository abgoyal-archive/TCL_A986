

#include "config.h"
#include "XPathNamespace.h"

#if ENABLE(XPATH)

#include "Element.h"

namespace WebCore {

XPathNamespace::XPathNamespace(PassRefPtr<Element> ownerElement, const AtomicString& prefix, const AtomicString& uri)
    : Node(ownerElement->document(), CreateOther)
    , m_ownerElement(ownerElement)
    , m_prefix(prefix)
    , m_uri(uri)
{
}

Document* XPathNamespace::ownerDocument() const
{
    return m_ownerElement->ownerDocument();
}

Element* XPathNamespace::ownerElement() const
{
    return m_ownerElement.get();
}

const AtomicString& XPathNamespace::prefix() const
{
    return m_prefix;
}

String XPathNamespace::nodeName() const
{
    return "#namespace";
}

String XPathNamespace::nodeValue() const
{
    return m_uri;
}

const AtomicString& XPathNamespace::namespaceURI() const
{
    return m_uri;
}

Node::NodeType XPathNamespace::nodeType() const
{
    return Node::XPATH_NAMESPACE_NODE;
}

}

#endif // ENABLE(XPATH)
