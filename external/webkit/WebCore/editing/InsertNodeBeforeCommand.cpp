

#include "config.h"
#include "InsertNodeBeforeCommand.h"

#include "htmlediting.h"

namespace WebCore {

InsertNodeBeforeCommand::InsertNodeBeforeCommand(PassRefPtr<Node> insertChild, PassRefPtr<Node> refChild)
    : SimpleEditCommand(refChild->document())
    , m_insertChild(insertChild)
    , m_refChild(refChild)
{
    ASSERT(m_insertChild);
    ASSERT(!m_insertChild->parentNode());
    ASSERT(m_refChild);
    ASSERT(m_refChild->parentNode());

    ASSERT(m_refChild->parentNode()->isContentEditable() || !m_refChild->parentNode()->attached());
}

void InsertNodeBeforeCommand::doApply()
{
    Node* parent = m_refChild->parentNode();
    if (!parent || !parent->isContentEditable())
        return;

    ExceptionCode ec;
    parent->insertBefore(m_insertChild.get(), m_refChild.get(), ec);
}

void InsertNodeBeforeCommand::doUnapply()
{
    if (!m_insertChild->isContentEditable())
        return;
        
    ExceptionCode ec;
    m_insertChild->remove(ec);
}

}
