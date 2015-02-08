

#include "config.h"
#include "JSNodeFilter.h"

#include "JSNode.h"
#include "JSNodeFilterCondition.h"
#include "NodeFilter.h"
#include "JSDOMBinding.h"

using namespace JSC;

namespace WebCore {

void JSNodeFilter::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);
    impl()->markAggregate(markStack);
}

JSValue JSNodeFilter::acceptNode(ExecState* exec, const ArgList& args)
{
    return jsNumber(exec, impl()->acceptNode(exec, toNode(args.at(0))));
}

PassRefPtr<NodeFilter> toNodeFilter(JSValue value)
{
    if (value.inherits(&JSNodeFilter::s_info))
        return static_cast<JSNodeFilter*>(asObject(value))->impl();

    return NodeFilter::create(JSNodeFilterCondition::create(value));
}

} // namespace WebCore
