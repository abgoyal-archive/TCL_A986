

#include "config.h"
#include "MergeIdenticalElementsCommand.h"

#include "Element.h"

namespace WebCore {

MergeIdenticalElementsCommand::MergeIdenticalElementsCommand(PassRefPtr<Element> first, PassRefPtr<Element> second)
    : SimpleEditCommand(first->document())
    , m_element1(first)
    , m_element2(second)
{
    ASSERT(m_element1);
    ASSERT(m_element2);
    ASSERT(m_element1->nextSibling() == m_element2);
}

void MergeIdenticalElementsCommand::doApply()
{
    if (m_element1->nextSibling() != m_element2 || !m_element1->isContentEditable() || !m_element2->isContentEditable())
        return;

    m_atChild = m_element2->firstChild();

    ExceptionCode ec = 0;

    Vector<RefPtr<Node> > children;
    for (Node* child = m_element1->firstChild(); child; child = child->nextSibling())
        children.append(child);

    size_t size = children.size();
    for (size_t i = 0; i < size; ++i)
        m_element2->insertBefore(children[i].release(), m_atChild.get(), ec);

    m_element1->remove(ec);
}

void MergeIdenticalElementsCommand::doUnapply()
{
    ASSERT(m_element1);
    ASSERT(m_element2);

    RefPtr<Node> atChild = m_atChild.release();

    Node* parent = m_element2->parent();
    if (!parent || !parent->isContentEditable())
        return;

    ExceptionCode ec = 0;

    parent->insertBefore(m_element1.get(), m_element2.get(), ec);
    if (ec)
        return;

    Vector<RefPtr<Node> > children;
    for (Node* child = m_element2->firstChild(); child && child != atChild; child = child->nextSibling())
        children.append(child);

    size_t size = children.size();
    for (size_t i = 0; i < size; ++i)
        m_element1->appendChild(children[i].release(), ec);
}

} // namespace WebCore
