

#include "config.h"
#include "ReplaceNodeWithSpanCommand.h"

#include "htmlediting.h"
#include "HTMLElement.h"
#include "HTMLNames.h"
#include "NamedAttrMap.h"

#include <wtf/Assertions.h>

namespace WebCore {

using namespace HTMLNames;

ReplaceNodeWithSpanCommand::ReplaceNodeWithSpanCommand(PassRefPtr<Node> node)
    : CompositeEditCommand(node->document())
    , m_node(node)
{
    ASSERT(m_node);
}

static void swapInNodePreservingAttributesAndChildren(Node* newNode, Node* nodeToReplace)
{
    ASSERT(nodeToReplace->inDocument());
    ExceptionCode ec = 0;
    Node* parentNode = nodeToReplace->parentNode();
    parentNode->insertBefore(newNode, nodeToReplace, ec);
    ASSERT(!ec);

    Node* nextChild;
    for (Node* child = nodeToReplace->firstChild(); child; child = nextChild) {
        nextChild = child->nextSibling();
        newNode->appendChild(child, ec);
        ASSERT(!ec);
    }

    newNode->attributes()->setAttributes(*nodeToReplace->attributes());

    parentNode->removeChild(nodeToReplace, ec);
    ASSERT(!ec);
}

void ReplaceNodeWithSpanCommand::doApply()
{
    if (!m_node->inDocument())
        return;
    if (!m_spanElement)
        m_spanElement = createHTMLElement(m_node->document(), spanTag);
    swapInNodePreservingAttributesAndChildren(m_spanElement.get(), m_node.get());
}

void ReplaceNodeWithSpanCommand::doUnapply()
{
    if (!m_spanElement->inDocument())
        return;
    swapInNodePreservingAttributesAndChildren(m_node.get(), m_spanElement.get());
}

} // namespace WebCore
