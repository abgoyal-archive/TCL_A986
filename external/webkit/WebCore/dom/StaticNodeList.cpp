

#include "config.h"
#include "StaticNodeList.h"

#include "Element.h"

namespace WebCore {

unsigned StaticNodeList::length() const
{
    return m_nodes.size();
}

Node* StaticNodeList::item(unsigned index) const
{
    if (index < m_nodes.size())
        return m_nodes[index].get();
    return 0;
}

Node* StaticNodeList::itemWithName(const AtomicString& elementId) const
{
    size_t length = m_nodes.size();
    for (size_t i = 0; i < length; ++i) {
        Node* node = m_nodes[i].get();
        if (node->isElementNode() && static_cast<Element*>(node)->getIDAttribute() == elementId)
            return node;
    }

    return 0;
}

} // namespace WebCore
