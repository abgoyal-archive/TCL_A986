

#include "config.h"
#include "InsertIntoTextNodeCommand.h"

#include "Text.h"

namespace WebCore {

InsertIntoTextNodeCommand::InsertIntoTextNodeCommand(PassRefPtr<Text> node, unsigned offset, const String& text)
    : SimpleEditCommand(node->document())
    , m_node(node)
    , m_offset(offset)
    , m_text(text)
{
    ASSERT(m_node);
    ASSERT(m_offset <= m_node->length());
    ASSERT(!m_text.isEmpty());
}

void InsertIntoTextNodeCommand::doApply()
{
    if (!m_node->isContentEditable())
        return;
    
    ExceptionCode ec;
    m_node->insertData(m_offset, m_text, ec);
}

void InsertIntoTextNodeCommand::doUnapply()
{
    if (!m_node->isContentEditable())
        return;
        
    ExceptionCode ec;
    m_node->deleteData(m_offset, m_text.length(), ec);
}

} // namespace WebCore
