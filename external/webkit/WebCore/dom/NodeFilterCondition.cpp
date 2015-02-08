

#include "config.h"
#include "NodeFilterCondition.h"

#include "NodeFilter.h"

namespace WebCore {

short NodeFilterCondition::acceptNode(ScriptState*, Node*) const
{
    return NodeFilter::FILTER_ACCEPT;
}

} // namespace WebCore
