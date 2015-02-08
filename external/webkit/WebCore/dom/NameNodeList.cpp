

#include "config.h"
#include "NameNodeList.h"

#include "Element.h"
#include "HTMLNames.h"
#include <wtf/Assertions.h>

namespace WebCore {

using namespace HTMLNames;

NameNodeList::NameNodeList(PassRefPtr<Node> rootNode, const String& name, DynamicNodeList::Caches* caches)
    : DynamicNodeList(rootNode, caches)
    , m_nodeName(name)
{
}

bool NameNodeList::nodeMatches(Element* testNode) const
{
    return testNode->getAttribute(nameAttr) == m_nodeName;
}

} // namespace WebCore
