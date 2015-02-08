

#include "config.h"
#include "ScriptState.h"

#include "Frame.h"
#include "Node.h"
#include "Page.h"
#include "ScriptController.h"
#include "V8HiddenPropertyName.h"

#include <v8.h>
#include <wtf/Assertions.h>

namespace WebCore {

ScriptState::ScriptState(v8::Handle<v8::Context> context)
    : m_context(v8::Persistent<v8::Context>::New(context))
{
    m_context.MakeWeak(this, &ScriptState::weakReferenceCallback);
}

ScriptState::~ScriptState()
{
    m_context.Dispose();
    m_context.Clear();
}

ScriptState* ScriptState::forContext(v8::Local<v8::Context> context)
{
    v8::Context::Scope contextScope(context);

    v8::Local<v8::Object> global = context->Global();
    // Skip proxy object. The proxy object will survive page navigation while we need
    // an object whose lifetime consides with that of the inspected context.
    global = v8::Local<v8::Object>::Cast(global->GetPrototype());

    v8::Handle<v8::String> key = V8HiddenPropertyName::scriptState();
    v8::Local<v8::Value> val = global->GetHiddenValue(key);
    if (!val.IsEmpty() && val->IsExternal())
        return static_cast<ScriptState*>(v8::External::Cast(*val)->Value());

    ScriptState* state = new ScriptState(context);
    global->SetHiddenValue(key, v8::External::New(state));
    return state;
}

ScriptState* ScriptState::current()
{
    v8::HandleScope handleScope;
    v8::Local<v8::Context> context = v8::Context::GetCurrent();
    if (context.IsEmpty()) {
        ASSERT_NOT_REACHED();
        return 0;
    }
    return ScriptState::forContext(context);
}

void ScriptState::weakReferenceCallback(v8::Persistent<v8::Value> object, void* parameter)
{
    ScriptState* scriptState = static_cast<ScriptState*>(parameter);
    delete scriptState;
}

ScriptState* mainWorldScriptState(Frame* frame)
{
    v8::HandleScope handleScope;
    V8Proxy* proxy = frame->script()->proxy();
    return ScriptState::forContext(proxy->mainWorldContext());
}

ScriptState* scriptStateFromNode(DOMWrapperWorld*, Node* node)
{
    // This should be never reached with V8 bindings (WebKit only uses it
    // for non-JS bindings)
    ASSERT_NOT_REACHED();
    return 0;
}

ScriptState* scriptStateFromPage(DOMWrapperWorld*, Page* page)
{
    // This should be only reached with V8 bindings from single process layout tests.
    return mainWorldScriptState(page->mainFrame());
}

}
