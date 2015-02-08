

#include "config.h"
#include "ClassNodeList.h"

#include "Document.h"
#include "StyledElement.h"

namespace WebCore {

ClassNodeList::ClassNodeList(PassRefPtr<Node> rootNode, const String& classNames, DynamicNodeList::Caches* caches)
    : DynamicNodeList(rootNode, caches)
    , m_classNames(classNames, m_rootNode->document()->inCompatMode())
{
}

bool ClassNodeList::nodeMatches(Element* testNode) const
{
    if (!testNode->hasClass())
        return false;
    if (!m_classNames.size())
        return false;
    ASSERT(testNode->isStyledElement());
    return static_cast<StyledElement*>(testNode)->classNames().containsAll(m_classNames);
}

} // namespace WebCore
