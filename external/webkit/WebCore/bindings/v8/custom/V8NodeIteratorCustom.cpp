

#include "config.h"
#include "V8NodeIterator.h"

#include "NodeIterator.h"
#include "ScriptState.h"

#include "V8Binding.h"
#include "V8Node.h"
#include "V8Proxy.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

static inline v8::Handle<v8::Value> toV8(PassRefPtr<Node> object, ExceptionCode ec, ScriptState* state)
{
    if (ec)
        return throwError(ec);

    if (state->hadException())
        return throwError(state->exception());

    if (!object)
        return v8::Null();

    return toV8(object);
}

v8::Handle<v8::Value> V8NodeIterator::nextNodeCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.NodeIterator.nextNode()");
    NodeIterator* nodeIterator = V8NodeIterator::toNative(args.Holder());

    ExceptionCode ec = 0;
    EmptyScriptState state;
    RefPtr<Node> result = nodeIterator->nextNode(&state, ec);
    return toV8(result.release(), ec, &state);
}

v8::Handle<v8::Value> V8NodeIterator::previousNodeCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.NodeIterator.previousNode()");
    NodeIterator* nodeIterator = V8NodeIterator::toNative(args.Holder());

    ExceptionCode ec = 0;
    EmptyScriptState state;
    RefPtr<Node> result = nodeIterator->previousNode(&state, ec);
    return toV8(result.release(), ec, &state);
}

} // namespace WebCore
