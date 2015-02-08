

#include "config.h"
#include "JSNodeFilterCondition.h"

#include "JSNode.h"
#include "JSNodeFilter.h"
#include "NodeFilter.h"
#include <runtime/JSLock.h>

namespace WebCore {

using namespace JSC;

ASSERT_CLASS_FITS_IN_CELL(JSNodeFilterCondition);

JSNodeFilterCondition::JSNodeFilterCondition(JSValue filter)
    : m_filter(filter)
{
}

void JSNodeFilterCondition::markAggregate(MarkStack& markStack)
{
    markStack.append(m_filter);
}

short JSNodeFilterCondition::acceptNode(JSC::ExecState* exec, Node* filterNode) const
{
    JSLock lock(SilenceAssertionsOnly);

    CallData callData;
    CallType callType = m_filter.getCallData(callData);
    if (callType == CallTypeNone)
        return NodeFilter::FILTER_ACCEPT;

   // The exec argument here should only be null if this was called from a
   // non-JavaScript language, and this is a JavaScript filter, and the document
   // in question is not associated with the frame. In that case, we're going to
   // behave incorrectly, and just reject nodes instead of calling the filter function.
   // To fix that we'd need to come up with a way to find a suitable JavaScript
   // execution context for the filter function to run in.
    if (!exec)
        return NodeFilter::FILTER_REJECT;

    MarkedArgumentBuffer args;
    // FIXME: The node should have the prototype chain that came from its document, not
    // whatever prototype chain might be on the window this filter came from. Bug 27662
    args.append(toJS(exec, deprecatedGlobalObjectForPrototype(exec), filterNode));
    if (exec->hadException())
        return NodeFilter::FILTER_REJECT;

    JSValue result = JSC::call(exec, m_filter, callType, callData, m_filter, args);
    if (exec->hadException())
        return NodeFilter::FILTER_REJECT;

    int intResult = result.toInt32(exec);
    if (exec->hadException())
        return NodeFilter::FILTER_REJECT;

    return intResult;
}

} // namespace WebCore
