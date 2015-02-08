

#include "config.h"
#include "JoinTextNodesCommand.h"

#include "Text.h"

namespace WebCore {

JoinTextNodesCommand::JoinTextNodesCommand(PassRefPtr<Text> text1, PassRefPtr<Text> text2)
    : SimpleEditCommand(text1->document()), m_text1(text1), m_text2(text2)
{
    ASSERT(m_text1);
    ASSERT(m_text2);
    ASSERT(m_text1->nextSibling() == m_text2);
    ASSERT(m_text1->length() > 0);
    ASSERT(m_text2->length() > 0);
}

void JoinTextNodesCommand::doApply()
{
    if (m_text1->nextSibling() != m_text2)
        return;

    Node* parent = m_text2->parentNode();
    if (!parent || !parent->isContentEditable())
        return;
    
    ExceptionCode ec = 0;
    m_text2->insertData(0, m_text1->data(), ec);
    if (ec)
        return;

    m_text1->remove(ec);
}

void JoinTextNodesCommand::doUnapply()
{
    if (m_text1->parentNode())
        return;

    Node* parent = m_text2->parentNode();
    if (!parent || !parent->isContentEditable())
        return;

    ExceptionCode ec = 0;

    parent->insertBefore(m_text1.get(), m_text2.get(), ec);
    if (ec)
        return;

    m_text2->deleteData(0, m_text1->length(), ec);
}

} // namespace WebCore
