

#include "config.h"
#include "TagNodeList.h"

#include "Element.h"
#include <wtf/Assertions.h>

namespace WebCore {

TagNodeList::TagNodeList(PassRefPtr<Node> rootNode, const AtomicString& namespaceURI, const AtomicString& localName, DynamicNodeList::Caches* caches)
    : DynamicNodeList(rootNode, caches)
    , m_namespaceURI(namespaceURI)
    , m_localName(localName)
{
    ASSERT(m_namespaceURI.isNull() || !m_namespaceURI.isEmpty());
}

bool TagNodeList::nodeMatches(Element* testNode) const
{
    if (m_namespaceURI != starAtom && m_namespaceURI != testNode->namespaceURI())
        return false;

    return m_localName == starAtom || m_localName == testNode->localName();
}

} // namespace WebCore
