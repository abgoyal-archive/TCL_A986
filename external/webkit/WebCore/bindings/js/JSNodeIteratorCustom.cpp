

#include "config.h"
#include "JSNodeIterator.h"

#include "JSNode.h"
#include "Node.h"
#include "NodeFilter.h"
#include "NodeIterator.h"

using namespace JSC;

namespace WebCore {

void JSNodeIterator::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    if (NodeFilter* filter = m_impl->filter())
        filter->markAggregate(markStack);
}

JSValue JSNodeIterator::nextNode(ExecState* exec, const ArgList&)
{
    ExceptionCode ec = 0;
    RefPtr<Node> node = impl()->nextNode(exec, ec);
    if (ec) {
        setDOMException(exec, ec);
        return jsUndefined();
    }

    if (exec->hadException())
        return jsUndefined();

    return toJS(exec, node.get());
}

JSValue JSNodeIterator::previousNode(ExecState* exec, const ArgList&)
{
    ExceptionCode ec = 0;
    RefPtr<Node> node = impl()->previousNode(exec, ec);
    if (ec) {
        setDOMException(exec, ec);
        return jsUndefined();
    }

    if (exec->hadException())
        return jsUndefined();

    return toJS(exec, node.get());
}

}
