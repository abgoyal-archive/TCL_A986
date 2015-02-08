

#include "config.h"
#include "SplitTextNodeContainingElementCommand.h"

#include "Element.h"
#include "Text.h"
#include "RenderObject.h"
#include <wtf/Assertions.h>

namespace WebCore {

SplitTextNodeContainingElementCommand::SplitTextNodeContainingElementCommand(PassRefPtr<Text> text, int offset)
    : CompositeEditCommand(text->document()), m_text(text), m_offset(offset)
{
    ASSERT(m_text);
    ASSERT(m_text->length() > 0);
}

void SplitTextNodeContainingElementCommand::doApply()
{
    ASSERT(m_text);
    ASSERT(m_offset > 0);

    splitTextNode(m_text.get(), m_offset);

    Element* parent = m_text->parentElement();
    if (!parent)
        return;

    RenderObject* parentRenderer = parent->renderer();
    if (!parentRenderer || !parentRenderer->isInline()) {
        wrapContentsInDummySpan(parent);
        Node* firstChild = parent->firstChild();
        if (!firstChild || !firstChild->isElementNode())
            return;
        parent = static_cast<Element*>(firstChild);
    }

    splitElement(parent, m_text);
}

}
