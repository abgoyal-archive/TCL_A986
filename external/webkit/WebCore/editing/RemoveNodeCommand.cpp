

#include "config.h"
#include "RemoveNodeCommand.h"

#include "Node.h"
#include <wtf/Assertions.h>

namespace WebCore {

RemoveNodeCommand::RemoveNodeCommand(PassRefPtr<Node> node)
    : SimpleEditCommand(node->document())
    , m_node(node)
{
    ASSERT(m_node);
    ASSERT(m_node->parentNode());
}

void RemoveNodeCommand::doApply()
{
    Node* parent = m_node->parentNode();
    if (!parent || !parent->isContentEditable())
        return;

    m_parent = parent;
    m_refChild = m_node->nextSibling();

    ExceptionCode ec;
    m_node->remove(ec);
}

void RemoveNodeCommand::doUnapply()
{
    RefPtr<Node> parent = m_parent.release();
    RefPtr<Node> refChild = m_refChild.release();
    if (!parent || !parent->isContentEditable())
        return;

    ExceptionCode ec;
    parent->insertBefore(m_node.get(), refChild.get(), ec);
}

}
