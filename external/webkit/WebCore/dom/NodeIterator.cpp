

#include "config.h"
#include "NodeIterator.h"

#include "Document.h"
#include "ExceptionCode.h"
#include "NodeFilter.h"
#include "ScriptState.h"

namespace WebCore {

NodeIterator::NodePointer::NodePointer()
{
}

NodeIterator::NodePointer::NodePointer(PassRefPtr<Node> n, bool b)
    : node(n)
    , isPointerBeforeNode(b)
{
}

void NodeIterator::NodePointer::clear()
{
    node.clear();
}

bool NodeIterator::NodePointer::moveToNext(Node* root)
{
    if (!node)
        return false;
    if (isPointerBeforeNode) {
        isPointerBeforeNode = false;
        return true;
    }
    node = node->traverseNextNode(root);
    return node;
}

bool NodeIterator::NodePointer::moveToPrevious(Node* root)
{
    if (!node)
        return false;
    if (!isPointerBeforeNode) {
        isPointerBeforeNode = true;
        return true;
    }
    node = node->traversePreviousNode(root);
    return node;
}

NodeIterator::NodeIterator(PassRefPtr<Node> rootNode, unsigned whatToShow, PassRefPtr<NodeFilter> filter, bool expandEntityReferences)
    : Traversal(rootNode, whatToShow, filter, expandEntityReferences)
    , m_referenceNode(root(), true)
    , m_detached(false)
{
    root()->document()->attachNodeIterator(this);
}

NodeIterator::~NodeIterator()
{
    root()->document()->detachNodeIterator(this);
}

PassRefPtr<Node> NodeIterator::nextNode(ScriptState* state, ExceptionCode& ec)
{
    if (m_detached) {
        ec = INVALID_STATE_ERR;
        return 0;
    }

    RefPtr<Node> result;

    m_candidateNode = m_referenceNode;
    while (m_candidateNode.moveToNext(root())) {
        // NodeIterators treat the DOM tree as a flat list of nodes.
        // In other words, FILTER_REJECT does not pass over descendants
        // of the rejected node. Hence, FILTER_REJECT is the same as FILTER_SKIP.
        RefPtr<Node> provisionalResult = m_candidateNode.node;
        bool nodeWasAccepted = acceptNode(state, provisionalResult.get()) == NodeFilter::FILTER_ACCEPT;
        if (state && state->hadException())
            break;
        if (nodeWasAccepted) {
            m_referenceNode = m_candidateNode;
            result = provisionalResult.release();
            break;
        }
    }

    m_candidateNode.clear();
    return result.release();
}

PassRefPtr<Node> NodeIterator::previousNode(ScriptState* state, ExceptionCode& ec)
{
    if (m_detached) {
        ec = INVALID_STATE_ERR;
        return 0;
    }

    RefPtr<Node> result;

    m_candidateNode = m_referenceNode;
    while (m_candidateNode.moveToPrevious(root())) {
        // NodeIterators treat the DOM tree as a flat list of nodes.
        // In other words, FILTER_REJECT does not pass over descendants
        // of the rejected node. Hence, FILTER_REJECT is the same as FILTER_SKIP.
        RefPtr<Node> provisionalResult = m_candidateNode.node;
        bool nodeWasAccepted = acceptNode(state, provisionalResult.get()) == NodeFilter::FILTER_ACCEPT;
        if (state && state->hadException())
            break;
        if (nodeWasAccepted) {
            m_referenceNode = m_candidateNode;
            result = provisionalResult.release();
            break;
        }
    }

    m_candidateNode.clear();
    return result.release();
}

void NodeIterator::detach()
{
    root()->document()->detachNodeIterator(this);
    m_detached = true;
    m_referenceNode.node.clear();
}

void NodeIterator::nodeWillBeRemoved(Node* removedNode)
{
    updateForNodeRemoval(removedNode, m_candidateNode);
    updateForNodeRemoval(removedNode, m_referenceNode);
}

void NodeIterator::updateForNodeRemoval(Node* removedNode, NodePointer& referenceNode) const
{
    ASSERT(!m_detached);
    ASSERT(removedNode);
    ASSERT(root()->document() == removedNode->document());

    // Iterator is not affected if the removed node is the reference node and is the root.
    // or if removed node is not the reference node, or the ancestor of the reference node.
    if (!removedNode->isDescendantOf(root()))
        return;
    bool willRemoveReferenceNode = removedNode == referenceNode.node;
    bool willRemoveReferenceNodeAncestor = referenceNode.node && referenceNode.node->isDescendantOf(removedNode);
    if (!willRemoveReferenceNode && !willRemoveReferenceNodeAncestor)
        return;

    if (referenceNode.isPointerBeforeNode) {
        Node* node = removedNode->traverseNextNode(root());
        if (node) {
            // Move out from under the node being removed if the reference node is
            // a descendant of the node being removed.
            if (willRemoveReferenceNodeAncestor) {
                while (node && node->isDescendantOf(removedNode))
                    node = node->traverseNextNode(root());
            }
            if (node)
                referenceNode.node = node;
        } else {
            node = removedNode->traversePreviousNode(root());
            if (node) {
                // Move out from under the node being removed if the reference node is
                // a descendant of the node being removed.
                if (willRemoveReferenceNodeAncestor) {
                    while (node && node->isDescendantOf(removedNode))
                        node = node->traversePreviousNode(root());
                }
                if (node) {
                    // Removing last node.
                    // Need to move the pointer after the node preceding the 
                    // new reference node.
                    referenceNode.node = node;
                    referenceNode.isPointerBeforeNode = false;
                }
            }
        }
    } else {
        Node* node = removedNode->traversePreviousNode(root());
        if (node) {
            // Move out from under the node being removed if the reference node is
            // a descendant of the node being removed.
            if (willRemoveReferenceNodeAncestor) {
                while (node && node->isDescendantOf(removedNode))
                    node = node->traversePreviousNode(root());
            }
            if (node)
                referenceNode.node = node;
        } else {
            node = removedNode->traverseNextNode(root());
            // Move out from under the node being removed if the reference node is
            // a descendant of the node being removed.
            if (willRemoveReferenceNodeAncestor) {
                while (node && node->isDescendantOf(removedNode))
                    node = node->traversePreviousNode(root());
            }
            if (node)
                referenceNode.node = node;
        }
    }
}


} // namespace WebCore
