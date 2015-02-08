

#include "config.h"
#include "ChildNodeList.h"

#include "Element.h"

namespace WebCore {

ChildNodeList::ChildNodeList(PassRefPtr<Node> rootNode, DynamicNodeList::Caches* info)
    : DynamicNodeList(rootNode, info)
{
}

unsigned ChildNodeList::length() const
{
    if (m_caches->isLengthCacheValid)
        return m_caches->cachedLength;

    unsigned len = 0;
    for (Node* n = m_rootNode->firstChild(); n; n = n->nextSibling())
        len++;

    m_caches->cachedLength = len;
    m_caches->isLengthCacheValid = true;

    return len;
}

Node* ChildNodeList::item(unsigned index) const
{
    unsigned int pos = 0;
    Node* n = m_rootNode->firstChild();

    if (m_caches->isItemCacheValid) {
        if (index == m_caches->lastItemOffset)
            return m_caches->lastItem;
        
        int diff = index - m_caches->lastItemOffset;
        unsigned dist = abs(diff);
        if (dist < index) {
            n = m_caches->lastItem;
            pos = m_caches->lastItemOffset;
        }
    }

    if (m_caches->isLengthCacheValid) {
        if (index >= m_caches->cachedLength)
            return 0;

        int diff = index - pos;
        unsigned dist = abs(diff);
        if (dist > m_caches->cachedLength - 1 - index) {
            n = m_rootNode->lastChild();
            pos = m_caches->cachedLength - 1;
        }
    }

    if (pos <= index) {
        while (n && pos < index) {
            n = n->nextSibling();
            ++pos;
        }
    } else {
        while (n && pos > index) {
            n = n->previousSibling();
            --pos;
        }
    }

    if (n) {
        m_caches->lastItem = n;
        m_caches->lastItemOffset = pos;
        m_caches->isItemCacheValid = true;
        return n;
    }

    return 0;
}

bool ChildNodeList::nodeMatches(Element* testNode) const
{
    // Note: Due to the overrides of the length and item functions above,
    // this function will be called only by DynamicNodeList::itemWithName,
    // for an element that was located with getElementById.
    return testNode->parentNode() == m_rootNode;
}

} // namespace WebCore
