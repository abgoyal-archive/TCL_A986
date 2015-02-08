

#ifndef ContainerNodeAlgorithms_h
#define ContainerNodeAlgorithms_h

#include <wtf/Assertions.h>

namespace WebCore {

class Node;

namespace Private {

    template<class GenericNode, class GenericNodeContainer>
    void addChildNodesToDeletionQueue(GenericNode*& head, GenericNode*& tail, GenericNodeContainer* container);

};

// Helper functions for TreeShared-derived classes, which have a 'Node' style interface
// This applies to 'ContainerNode' and 'SVGElementInstance'
template<class GenericNode, class GenericNodeContainer>
void removeAllChildrenInContainer(GenericNodeContainer* container)
{
    // List of nodes to be deleted.
    GenericNode* head = 0;
    GenericNode* tail = 0;

    Private::addChildNodesToDeletionQueue<GenericNode, GenericNodeContainer>(head, tail, container);

    GenericNode* n;
    GenericNode* next;
    while ((n = head) != 0) {
        ASSERT(n->m_deletionHasBegun);

        next = n->nextSibling();
        n->setNextSibling(0);

        head = next;
        if (next == 0)
            tail = 0;

        if (n->hasChildNodes())
            Private::addChildNodesToDeletionQueue<GenericNode, GenericNodeContainer>(head, tail, static_cast<GenericNodeContainer*>(n));

        delete n;
    }
}

template<class GenericNode, class GenericNodeContainer>
void appendChildToContainer(GenericNode* child, GenericNodeContainer* container)
{
    child->setParent(container);

    GenericNode* lastChild = container->lastChild();
    if (lastChild) {
        child->setPreviousSibling(lastChild);
        lastChild->setNextSibling(child);
    } else
        container->setFirstChild(child);

    container->setLastChild(child);
}

// Helper methods for removeAllChildrenInContainer, hidden from WebCore namespace
namespace Private {

    template<class GenericNode, bool dispatchRemovalNotification>
    struct NodeRemovalDispatcher {
        static void dispatch(GenericNode*)
        {
            // no-op, by default
        }
    };

    template<class GenericNode>
    struct NodeRemovalDispatcher<GenericNode, true> {
        static void dispatch(GenericNode* node)
        {
            if (node->inDocument())
                node->removedFromDocument();
        }
    };

    template<class GenericNode>
    struct ShouldDispatchRemovalNotification {
        static const bool value = false;
    };

    template<>
    struct ShouldDispatchRemovalNotification<Node> {
        static const bool value = true;
    };

    template<class GenericNode, class GenericNodeContainer>
    void addChildNodesToDeletionQueue(GenericNode*& head, GenericNode*& tail, GenericNodeContainer* container)
    {
        // We have to tell all children that their parent has died.
        GenericNode* next = 0;
        for (GenericNode* n = container->firstChild(); n != 0; n = next) {
            ASSERT(!n->m_deletionHasBegun);

            next = n->nextSibling();
            n->setPreviousSibling(0);
            n->setNextSibling(0);
            n->setParent(0);

            if (!n->refCount()) {
#ifndef NDEBUG
                n->m_deletionHasBegun = true;
#endif
                // Add the node to the list of nodes to be deleted.
                // Reuse the nextSibling pointer for this purpose.
                if (tail)
                    tail->setNextSibling(n);
                else
                    head = n;

                tail = n;
            } else
                NodeRemovalDispatcher<GenericNode, ShouldDispatchRemovalNotification<GenericNode>::value>::dispatch(n);
        }

        container->setFirstChild(0);
        container->setLastChild(0);
    }
};

} // namespace WebCore

#endif // ContainerNodeAlgorithms_h
