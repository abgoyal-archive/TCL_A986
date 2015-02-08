

#include "config.h"
#include "SplitTextNodeCommand.h"

#include "Document.h"
#include "Text.h"
#include <wtf/Assertions.h>

namespace WebCore {

SplitTextNodeCommand::SplitTextNodeCommand(PassRefPtr<Text> text, int offset)
    : SimpleEditCommand(text->document())
    , m_text2(text)
    , m_offset(offset)
{
    // NOTE: Various callers rely on the fact that the original node becomes
    // the second node (i.e. the new node is inserted before the existing one).
    // That is not a fundamental dependency (i.e. it could be re-coded), but
    // rather is based on how this code happens to work.
    ASSERT(m_text2);
    ASSERT(m_text2->length() > 0);
    ASSERT(m_offset > 0);
    ASSERT(m_offset < m_text2->length());
}

void SplitTextNodeCommand::doApply()
{
    Node* parent = m_text2->parentNode();
    if (!parent || !parent->isContentEditable())
        return;

    ExceptionCode ec = 0;

    String prefixText = m_text2->substringData(0, m_offset, ec);
    if (prefixText.isEmpty())
        return;

    RefPtr<Text> prefixTextNode = Text::create(document(), prefixText);
    ASSERT(prefixTextNode);
    document()->copyMarkers(m_text2.get(), 0, m_offset, prefixTextNode.get(), 0);

    parent->insertBefore(prefixTextNode.get(), m_text2.get(), ec);
    if (ec)
        return;

    m_text2->deleteData(0, m_offset, ec);
    m_text1 = prefixTextNode.release();
}

void SplitTextNodeCommand::doUnapply()
{
    if (!m_text1 || !m_text1->isContentEditable())
        return;

    ASSERT(m_text1->document() == document());

    RefPtr<Text> prefixTextNode = m_text1.release();
    String prefixText = prefixTextNode->data();

    ExceptionCode ec = 0;
    m_text2->insertData(0, prefixText, ec);
    if (ec)
        return;

    document()->copyMarkers(prefixTextNode.get(), 0, prefixText.length(), m_text2.get(), 0);
    prefixTextNode->remove(ec);
}

} // namespace WebCore
