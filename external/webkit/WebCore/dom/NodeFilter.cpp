

#include "config.h"
#include "NodeFilter.h"

#include "Node.h"

namespace WebCore {

short NodeFilter::acceptNode(ScriptState* state, Node* node) const
{
    // cast to short silences "enumeral and non-enumeral types in return" warning
    return m_condition ? m_condition->acceptNode(state, node) : static_cast<short>(FILTER_ACCEPT);
}

} // namespace WebCore
