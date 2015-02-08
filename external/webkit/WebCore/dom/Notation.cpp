

#include "config.h"
#include "Notation.h"

namespace WebCore {

Notation::Notation(Document* document, const String& name, const String& publicId, const String& systemId)
    : ContainerNode(document)
    , m_name(name)
    , m_publicId(publicId)
    , m_systemId(systemId)
{
}

String Notation::nodeName() const
{
    return m_name;
}

Node::NodeType Notation::nodeType() const
{
    return NOTATION_NODE;
}

PassRefPtr<Node> Notation::cloneNode(bool /*deep*/)
{
    // Spec says cloning Notation nodes is "implementation dependent". We do not support it.
    return 0;
}

bool Notation::childTypeAllowed(NodeType)
{
    return false;
}

} // namespace
