

#include "config.h"
#include "JSJavaScriptCallFrame.h"

#if ENABLE(JAVASCRIPT_DEBUGGER)

#include "JavaScriptCallFrame.h"
#include <runtime/ArrayPrototype.h>

using namespace JSC;

namespace WebCore {

JSValue JSJavaScriptCallFrame::evaluate(ExecState* exec, const ArgList& args)
{
    JSValue exception;
    JSValue result = impl()->evaluate(args.at(0).toString(exec), exception);

    if (exception)
        exec->setException(exception);

    return result;
}

JSValue JSJavaScriptCallFrame::thisObject(ExecState*) const
{
    return impl()->thisObject() ? impl()->thisObject() : jsNull();
}

JSValue JSJavaScriptCallFrame::type(ExecState* exec) const
{
    switch (impl()->type()) {
        case DebuggerCallFrame::FunctionType:
            return jsString(exec, UString("function"));
        case DebuggerCallFrame::ProgramType:
            return jsString(exec, UString("program"));
    }

    ASSERT_NOT_REACHED();
    return jsNull();
}

JSValue JSJavaScriptCallFrame::scopeChain(ExecState* exec) const
{
    if (!impl()->scopeChain())
        return jsNull();

    const ScopeChainNode* scopeChain = impl()->scopeChain();
    ScopeChainIterator iter = scopeChain->begin();
    ScopeChainIterator end = scopeChain->end();

    // we must always have something in the scope chain
    ASSERT(iter != end);

    MarkedArgumentBuffer list;
    do {
        list.append(*iter);
        ++iter;
    } while (iter != end);

    return constructArray(exec, list);
}

} // namespace WebCore

#endif // ENABLE(JAVASCRIPT_DEBUGGER)
