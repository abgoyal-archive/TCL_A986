

#include "config.h"
#include "WrapContentsInDummySpanCommand.h"

#include "ApplyStyleCommand.h"
#include "HTMLElement.h"

namespace WebCore {

WrapContentsInDummySpanCommand::WrapContentsInDummySpanCommand(PassRefPtr<Element> element)
    : SimpleEditCommand(element->document())
    , m_element(element)
{
    ASSERT(m_element);
}

void WrapContentsInDummySpanCommand::executeApply()
{
    Vector<RefPtr<Node> > children;
    for (Node* child = m_element->firstChild(); child; child = child->nextSibling())
        children.append(child);
    
    ExceptionCode ec;
    
    size_t size = children.size();
    for (size_t i = 0; i < size; ++i)
        m_dummySpan->appendChild(children[i].release(), ec);
    
    m_element->appendChild(m_dummySpan.get(), ec);
}

void WrapContentsInDummySpanCommand::doApply()
{
    m_dummySpan = createStyleSpanElement(document());
    
    executeApply();
}
    
void WrapContentsInDummySpanCommand::doUnapply()
{
    ASSERT(m_element);

    if (!m_dummySpan || !m_element->isContentEditable())
        return;

    Vector<RefPtr<Node> > children;
    for (Node* child = m_dummySpan->firstChild(); child; child = child->nextSibling())
        children.append(child);

    ExceptionCode ec;

    size_t size = children.size();
    for (size_t i = 0; i < size; ++i)
        m_element->appendChild(children[i].release(), ec);

    m_dummySpan->remove(ec);
}

void WrapContentsInDummySpanCommand::doReapply()
{
    ASSERT(m_element);
    
    if (!m_dummySpan || !m_element->isContentEditable())
        return;

    executeApply();
}
    
} // namespace WebCore
