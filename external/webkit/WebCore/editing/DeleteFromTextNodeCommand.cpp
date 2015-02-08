

#include "config.h"
#include "DeleteFromTextNodeCommand.h"

#include "Text.h"

namespace WebCore {

DeleteFromTextNodeCommand::DeleteFromTextNodeCommand(PassRefPtr<Text> node, unsigned offset, unsigned count)
    : SimpleEditCommand(node->document())
    , m_node(node)
    , m_offset(offset)
    , m_count(count)
{
    ASSERT(m_node);
    ASSERT(m_offset <= m_node->length());
    ASSERT(m_offset + m_count <= m_node->length());
}

void DeleteFromTextNodeCommand::doApply()
{
    ASSERT(m_node);

    if (!m_node->isContentEditable())
        return;

    ExceptionCode ec = 0;
    m_text = m_node->substringData(m_offset, m_count, ec);
    if (ec)
        return;
    
    m_node->deleteData(m_offset, m_count, ec);
}

void DeleteFromTextNodeCommand::doUnapply()
{
    ASSERT(m_node);

    if (!m_node->isContentEditable())
        return;
        
    ExceptionCode ec;
    m_node->insertData(m_offset, m_text, ec);
}

} // namespace WebCore
