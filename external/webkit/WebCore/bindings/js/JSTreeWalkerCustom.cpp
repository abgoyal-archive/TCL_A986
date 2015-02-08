

#include "config.h"
#include "JSTreeWalker.h"

#include "JSNode.h"
#include "Node.h"
#include "NodeFilter.h"
#include "TreeWalker.h"

using namespace JSC;

namespace WebCore {
    
void JSTreeWalker::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    if (NodeFilter* filter = m_impl->filter())
        filter->markAggregate(markStack);
}
    
JSValue JSTreeWalker::parentNode(ExecState* exec, const ArgList&)
{
    Node* node = impl()->parentNode(exec);
    if (exec->hadException())
        return jsUndefined();
    return toJS(exec, node);
}
    
JSValue JSTreeWalker::firstChild(ExecState* exec, const ArgList&)
{
    Node* node = impl()->firstChild(exec);
    if (exec->hadException())
        return jsUndefined();
    return toJS(exec, node);
}
    
JSValue JSTreeWalker::lastChild(ExecState* exec, const ArgList&)
{
    Node* node = impl()->lastChild(exec);
    if (exec->hadException())
        return jsUndefined();
    return toJS(exec, node);
}
    
JSValue JSTreeWalker::nextSibling(ExecState* exec, const ArgList&)
{
    Node* node = impl()->nextSibling(exec);
    if (exec->hadException())
        return jsUndefined();
    return toJS(exec, node);
}
    
JSValue JSTreeWalker::previousSibling(ExecState* exec, const ArgList&)
{
    Node* node = impl()->previousSibling(exec);
    if (exec->hadException())
        return jsUndefined();
    return toJS(exec, node);
}
    
JSValue JSTreeWalker::previousNode(ExecState* exec, const ArgList&)
{
    Node* node = impl()->previousNode(exec);
    if (exec->hadException())
        return jsUndefined();
    return toJS(exec, node);
}
    
JSValue JSTreeWalker::nextNode(ExecState* exec, const ArgList&)
{
    Node* node = impl()->nextNode(exec);
    if (exec->hadException())
        return jsUndefined();
    return toJS(exec, node);
}

}
