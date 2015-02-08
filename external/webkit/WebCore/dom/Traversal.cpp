

#include "config.h"
#include "Traversal.h"

#include "Node.h"
#include "NodeFilter.h"

namespace WebCore {

Traversal::Traversal(PassRefPtr<Node> rootNode, unsigned whatToShow, PassRefPtr<NodeFilter> nodeFilter, bool expandEntityReferences)
    : m_root(rootNode)
    , m_whatToShow(whatToShow)
    , m_filter(nodeFilter)
    , m_expandEntityReferences(expandEntityReferences)
{
}

short Traversal::acceptNode(ScriptState* state, Node* node) const
{
    // FIXME: To handle XML properly we would have to check m_expandEntityReferences.

    // The bit twiddling here is done to map DOM node types, which are given as integers from
    // 1 through 12, to whatToShow bit masks.
    if (!(((1 << (node->nodeType() - 1)) & m_whatToShow)))
        return NodeFilter::FILTER_SKIP;
    if (!m_filter)
        return NodeFilter::FILTER_ACCEPT;
    return m_filter->acceptNode(state, node);
}

} // namespace WebCore
