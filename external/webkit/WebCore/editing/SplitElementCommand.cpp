

#include "config.h"
#include "SplitElementCommand.h"

#include "Element.h"
#include <wtf/Assertions.h>

namespace WebCore {

SplitElementCommand::SplitElementCommand(PassRefPtr<Element> element, PassRefPtr<Node> atChild)
    : SimpleEditCommand(element->document())
    , m_element2(element)
    , m_atChild(atChild)
{
    ASSERT(m_element2);
    ASSERT(m_atChild);
    ASSERT(m_atChild->parentNode() == m_element2);
}

void SplitElementCommand::executeApply()
{
    if (m_atChild->parentNode() != m_element2)
        return;
    
    Vector<RefPtr<Node> > children;
    for (Node* node = m_element2->firstChild(); node != m_atChild; node = node->nextSibling())
        children.append(node);
    
    ExceptionCode ec = 0;
    
    Node* parent = m_element2->parentNode();
    if (!parent || !parent->isContentEditable())
        return;
    parent->insertBefore(m_element1.get(), m_element2.get(), ec);
    if (ec)
        return;
    
    size_t size = children.size();
    for (size_t i = 0; i < size; ++i)
        m_element1->appendChild(children[i], ec);
}
    
void SplitElementCommand::doApply()
{
    m_element1 = m_element2->cloneElementWithoutChildren();
    
    executeApply();
}

void SplitElementCommand::doUnapply()
{
    if (!m_element1 || !m_element1->isContentEditable() || !m_element2->isContentEditable())
        return;

    Vector<RefPtr<Node> > children;
    for (Node* node = m_element1->firstChild(); node; node = node->nextSibling())
        children.append(node);

    RefPtr<Node> refChild = m_element2->firstChild();

    ExceptionCode ec = 0;

    size_t size = children.size();
    for (size_t i = 0; i < size; ++i)
        m_element2->insertBefore(children[i].get(), refChild.get(), ec);

    m_element1->remove(ec);
}

void SplitElementCommand::doReapply()
{
    if (!m_element1)
        return;
    
    executeApply();
}
    
} // namespace WebCore
