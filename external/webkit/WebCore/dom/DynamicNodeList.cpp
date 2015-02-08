

#include "config.h"
#include "DynamicNodeList.h"

#include "Document.h"
#include "Element.h"

namespace WebCore {

DynamicNodeList::DynamicNodeList(PassRefPtr<Node> rootNode)
    : m_rootNode(rootNode)
    , m_caches(Caches::create())
    , m_ownsCaches(true)
{
    m_rootNode->registerDynamicNodeList(this);
}    

DynamicNodeList::DynamicNodeList(PassRefPtr<Node> rootNode, DynamicNodeList::Caches* caches)
    : m_rootNode(rootNode)
    , m_caches(caches)
    , m_ownsCaches(false)
{
    m_rootNode->registerDynamicNodeList(this);
}    

DynamicNodeList::~DynamicNodeList()
{
    m_rootNode->unregisterDynamicNodeList(this);
}

unsigned DynamicNodeList::length() const
{
    if (m_caches->isLengthCacheValid)
        return m_caches->cachedLength;

    unsigned length = 0;

    for (Node* n = m_rootNode->firstChild(); n; n = n->traverseNextNode(m_rootNode.get()))
        length += n->isElementNode() && nodeMatches(static_cast<Element*>(n));

    m_caches->cachedLength = length;
    m_caches->isLengthCacheValid = true;

    return length;
}

Node* DynamicNodeList::itemForwardsFromCurrent(Node* start, unsigned offset, int remainingOffset) const
{
    ASSERT(remainingOffset >= 0);
    for (Node* n = start; n; n = n->traverseNextNode(m_rootNode.get())) {
        if (n->isElementNode() && nodeMatches(static_cast<Element*>(n))) {
            if (!remainingOffset) {
                m_caches->lastItem = n;
                m_caches->lastItemOffset = offset;
                m_caches->isItemCacheValid = true;
                return n;
            }
            --remainingOffset;
        }
    }

    return 0; // no matching node in this subtree
}

Node* DynamicNodeList::itemBackwardsFromCurrent(Node* start, unsigned offset, int remainingOffset) const
{
    ASSERT(remainingOffset < 0);
    for (Node* n = start; n; n = n->traversePreviousNode(m_rootNode.get())) {
        if (n->isElementNode() && nodeMatches(static_cast<Element*>(n))) {
            if (!remainingOffset) {
                m_caches->lastItem = n;
                m_caches->lastItemOffset = offset;
                m_caches->isItemCacheValid = true;
                return n;
            }
            ++remainingOffset;
        }
    }

    return 0; // no matching node in this subtree
}

Node* DynamicNodeList::item(unsigned offset) const
{
    int remainingOffset = offset;
    Node* start = m_rootNode->firstChild();
    if (m_caches->isItemCacheValid) {
        if (offset == m_caches->lastItemOffset)
            return m_caches->lastItem;
        else if (offset > m_caches->lastItemOffset || m_caches->lastItemOffset - offset < offset) {
            start = m_caches->lastItem;
            remainingOffset -= m_caches->lastItemOffset;
        }
    }

    if (remainingOffset < 0)
        return itemBackwardsFromCurrent(start, offset, remainingOffset);
    return itemForwardsFromCurrent(start, offset, remainingOffset);
}

Node* DynamicNodeList::itemWithName(const AtomicString& elementId) const
{
    if (m_rootNode->isDocumentNode() || m_rootNode->inDocument()) {
        Element* node = m_rootNode->document()->getElementById(elementId);
        if (node && nodeMatches(node)) {
            for (Node* p = node->parentNode(); p; p = p->parentNode()) {
                if (p == m_rootNode)
                    return node;
            }
        }
        if (!node)
            return 0;
        // In the case of multiple nodes with the same name, just fall through.
    }

    unsigned length = this->length();
    for (unsigned i = 0; i < length; i++) {
        Node* node = item(i);
        if (node->isElementNode() && static_cast<Element*>(node)->getIDAttribute() == elementId)
            return node;
    }

    return 0;
}

void DynamicNodeList::invalidateCache()
{
    // This should only be called for node lists that own their own caches.
    ASSERT(m_ownsCaches);
    m_caches->reset();
}

DynamicNodeList::Caches::Caches()
    : lastItem(0)
    , isLengthCacheValid(false)
    , isItemCacheValid(false)
{
}

PassRefPtr<DynamicNodeList::Caches> DynamicNodeList::Caches::create()
{
    return adoptRef(new Caches());
}

void DynamicNodeList::Caches::reset()
{
    lastItem = 0;
    isLengthCacheValid = false;
    isItemCacheValid = false;     
}

} // namespace WebCore
