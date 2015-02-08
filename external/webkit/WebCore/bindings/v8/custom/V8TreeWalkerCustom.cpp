

#include "config.h"
#include "V8TreeWalker.h"

#include "Node.h"
#include "ScriptState.h"
#include "TreeWalker.h"

#include "V8Binding.h"
#include "V8Node.h"
#include "V8Proxy.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

static inline v8::Handle<v8::Value> toV8Object(PassRefPtr<Node> object, ScriptState* state)
{
    if (state->hadException())
        return throwError(state->exception());

    if (!object)
        return v8::Null();

    return toV8(object);
}

v8::Handle<v8::Value> V8TreeWalker::parentNodeCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.TreeWalker.parentNode()");
    TreeWalker* treeWalker = V8TreeWalker::toNative(args.Holder());

    EmptyScriptState state;
    RefPtr<Node> result = treeWalker->parentNode(&state);
    return toV8Object(result.release(), &state);
}

v8::Handle<v8::Value> V8TreeWalker::firstChildCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.TreeWalker.firstChild()");
    TreeWalker* treeWalker = V8TreeWalker::toNative(args.Holder());

    EmptyScriptState state;
    RefPtr<Node> result = treeWalker->firstChild(&state);
    return toV8Object(result.release(), &state);
}

v8::Handle<v8::Value> V8TreeWalker::lastChildCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.TreeWalker.lastChild()");
    TreeWalker* treeWalker = V8TreeWalker::toNative(args.Holder());

    EmptyScriptState state;
    RefPtr<Node> result = treeWalker->lastChild(&state);
    return toV8Object(result.release(), &state);
}

v8::Handle<v8::Value> V8TreeWalker::nextNodeCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.TreeWalker.nextNode()");
    TreeWalker* treeWalker = V8TreeWalker::toNative(args.Holder());

    EmptyScriptState state;
    RefPtr<Node> result = treeWalker->nextNode(&state);
    return toV8Object(result.release(), &state);
}

v8::Handle<v8::Value> V8TreeWalker::previousNodeCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.TreeWalker.previousNode()");
    TreeWalker* treeWalker = V8TreeWalker::toNative(args.Holder());

    EmptyScriptState state;
    RefPtr<Node> result = treeWalker->previousNode(&state);
    return toV8Object(result.release(), &state);
}

v8::Handle<v8::Value> V8TreeWalker::nextSiblingCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.TreeWalker.nextSibling()");
    TreeWalker* treeWalker = V8TreeWalker::toNative(args.Holder());

    EmptyScriptState state;
    RefPtr<Node> result = treeWalker->nextSibling(&state);
    return toV8Object(result.release(), &state);
}

v8::Handle<v8::Value> V8TreeWalker::previousSiblingCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.TreeWalker.previousSibling()");
    TreeWalker* treeWalker = V8TreeWalker::toNative(args.Holder());

    EmptyScriptState state;
    RefPtr<Node> result = treeWalker->previousSibling(&state);
    return toV8Object(result.release(), &state);
}

} // namespace WebCore
