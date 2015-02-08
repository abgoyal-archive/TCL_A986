

#include "config.h"
#include "EntityReference.h"

namespace WebCore {

inline EntityReference::EntityReference(Document* document, const String& entityName)
    : ContainerNode(document)
    , m_entityName(entityName)
{
}

PassRefPtr<EntityReference> EntityReference::create(Document* document, const String& entityName)
{
    return adoptRef(new EntityReference(document, entityName));
}

String EntityReference::nodeName() const
{
    return m_entityName;
}

Node::NodeType EntityReference::nodeType() const
{
    return ENTITY_REFERENCE_NODE;
}

PassRefPtr<Node> EntityReference::cloneNode(bool)
{
    return create(document(), m_entityName);
}

} // namespace
