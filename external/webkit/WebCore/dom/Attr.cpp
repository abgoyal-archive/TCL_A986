
#include "config.h"
#include "Attr.h"

#include "Document.h"
#include "Element.h"
#include "ExceptionCode.h"
#include "Text.h"
#include "XMLNSNames.h"

namespace WebCore {

using namespace HTMLNames;

inline Attr::Attr(Element* element, Document* document, PassRefPtr<Attribute> attribute)
    : ContainerNode(document)
    , m_element(element)
    , m_attribute(attribute)
    , m_ignoreChildrenChanged(0)
    , m_specified(true)  
{
    ASSERT(!m_attribute->attr());
    m_attribute->m_impl = this;
}

PassRefPtr<Attr> Attr::create(Element* element, Document* document, PassRefPtr<Attribute> attribute)
{
    RefPtr<Attr> attr = adoptRef(new Attr(element, document, attribute));
    attr->createTextChild();
    return attr.release();
}

Attr::~Attr()
{
    ASSERT(m_attribute->attr() == this);
    m_attribute->m_impl = 0;
}

void Attr::createTextChild()
{
    ASSERT(refCount());
    if (!m_attribute->value().isEmpty()) {
        RefPtr<Text> textNode = document()->createTextNode(m_attribute->value().string());

        // This does everything appendChild() would do in this situation (assuming m_ignoreChildrenChanged was set),
        // but much more efficiently.
        textNode->setParent(this);
        setFirstChild(textNode.get());
        setLastChild(textNode.get());
    }
}

String Attr::nodeName() const
{
    return name();
}

Node::NodeType Attr::nodeType() const
{
    return ATTRIBUTE_NODE;
}

const AtomicString& Attr::localName() const
{
    return m_attribute->localName();
}

const AtomicString& Attr::namespaceURI() const
{
    return m_attribute->namespaceURI();
}

const AtomicString& Attr::prefix() const
{
    return m_attribute->prefix();
}

void Attr::setPrefix(const AtomicString& prefix, ExceptionCode& ec)
{
    ec = 0;
    checkSetPrefix(prefix, ec);
    if (ec)
        return;

    if ((prefix == xmlnsAtom && namespaceURI() != XMLNSNames::xmlnsNamespaceURI)
        || static_cast<Attr*>(this)->qualifiedName() == xmlnsAtom) {
        ec = NAMESPACE_ERR;
        return;
    }

    m_attribute->setPrefix(prefix.isEmpty() ? AtomicString() : prefix);
}

String Attr::nodeValue() const
{
    return value();
}

void Attr::setValue(const AtomicString& value)
{
    m_ignoreChildrenChanged++;
    removeChildren();
    m_attribute->setValue(value);
    createTextChild();
    m_ignoreChildrenChanged--;
}

void Attr::setValue(const AtomicString& value, ExceptionCode&)
{
    if (m_element && m_attribute->name() == m_element->idAttributeName())
        m_element->updateId(m_element->getIDAttribute(), value);

    setValue(value);

    if (m_element)
        m_element->attributeChanged(m_attribute.get());
}

void Attr::setNodeValue(const String& v, ExceptionCode& ec)
{
    setValue(v, ec);
}

PassRefPtr<Node> Attr::cloneNode(bool /*deep*/)
{
    RefPtr<Attr> clone = adoptRef(new Attr(0, document(), m_attribute->clone()));
    cloneChildNodes(clone.get());
    return clone.release();
}

// DOM Section 1.1.1
bool Attr::childTypeAllowed(NodeType type)
{
    switch (type) {
        case TEXT_NODE:
        case ENTITY_REFERENCE_NODE:
            return true;
        default:
            return false;
    }
}

void Attr::childrenChanged(bool changedByParser, Node* beforeChange, Node* afterChange, int childCountDelta)
{
    if (m_ignoreChildrenChanged > 0)
        return;
 
    Node::childrenChanged(changedByParser, beforeChange, afterChange, childCountDelta);

    // FIXME: We should include entity references in the value
    
    String val = "";
    for (Node *n = firstChild(); n; n = n->nextSibling()) {
        if (n->isTextNode())
            val += static_cast<Text *>(n)->data();
    }

    if (m_element && m_attribute->name() == m_element->idAttributeName())
        m_element->updateId(m_attribute->value(), val);

    m_attribute->setValue(val.impl());
    if (m_element)
        m_element->attributeChanged(m_attribute.get());
}

bool Attr::isId() const
{
    return qualifiedName().matches(m_element ? m_element->idAttributeName() : idAttr);
}

}
