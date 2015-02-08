

#include "config.h"
#include "AppendNodeCommand.h"

#include "htmlediting.h"

namespace WebCore {

AppendNodeCommand::AppendNodeCommand(PassRefPtr<Element> parent, PassRefPtr<Node> node)
    : SimpleEditCommand(parent->document())
    , m_parent(parent)
    , m_node(node)
{
    ASSERT(m_parent);
    ASSERT(m_node);
    ASSERT(!m_node->parent());

    ASSERT(m_parent->isContentEditable() || !m_parent->attached());
}

void AppendNodeCommand::doApply()
{
    if (!m_parent->isContentEditable() && m_parent->attached())
        return;
        
    ExceptionCode ec;
    m_parent->appendChild(m_node.get(), ec);
}

void AppendNodeCommand::doUnapply()
{
    if (!m_parent->isContentEditable())
        return;
        
    ExceptionCode ec;
    m_node->remove(ec);
}

} // namespace WebCore
