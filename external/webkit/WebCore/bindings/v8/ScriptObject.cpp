

#include "config.h"
#include "ScriptObject.h"

#include "ScriptScope.h"
#include "ScriptState.h"

#include "Document.h"
#include "Frame.h"
#include "V8Binding.h"
#include "V8InjectedScriptHost.h"
#include "V8InspectorBackend.h"
#include "V8InspectorFrontendHost.h"
#include "V8Proxy.h"

#include <v8.h>

namespace WebCore {

ScriptObject::ScriptObject(ScriptState* scriptState, v8::Handle<v8::Object> v8Object)
    : ScriptValue(v8Object)
    , m_scriptState(scriptState)
{
}

v8::Local<v8::Object> ScriptObject::v8Object() const
{
    ASSERT(v8Value()->IsObject());
    return v8::Local<v8::Object>(v8::Object::Cast(*v8Value()));
}

bool ScriptObject::set(const String& name, const String& value)
{
    ScriptScope scope(m_scriptState);
    v8Object()->Set(v8String(name), v8String(value));
    return scope.success();
}

bool ScriptObject::set(const char* name, const ScriptObject& value)
{
    if (value.scriptState() != m_scriptState) {
        ASSERT_NOT_REACHED();
        return false;
    }
    ScriptScope scope(m_scriptState);
    v8Object()->Set(v8::String::New(name), value.v8Value());
    return scope.success();
}

bool ScriptObject::set(const char* name, const String& value)
{
    ScriptScope scope(m_scriptState);
    v8Object()->Set(v8::String::New(name), v8String(value));
    return scope.success();
}

bool ScriptObject::set(const char* name, double value)
{
    ScriptScope scope(m_scriptState);
    v8Object()->Set(v8::String::New(name), v8::Number::New(value));
    return scope.success();
}

bool ScriptObject::set(const char* name, long value)
{
    ScriptScope scope(m_scriptState);
    v8Object()->Set(v8::String::New(name), v8::Number::New(value));
    return scope.success();
}

bool ScriptObject::set(const char* name, long long value)
{
    ScriptScope scope(m_scriptState);
    v8Object()->Set(v8::String::New(name), v8::Number::New(value));
    return scope.success();
}

bool ScriptObject::set(const char* name, int value)
{
    ScriptScope scope(m_scriptState);
    v8Object()->Set(v8::String::New(name), v8::Number::New(value));
    return scope.success();
}

bool ScriptObject::set(const char* name, unsigned value)
{
    ScriptScope scope(m_scriptState);
    v8Object()->Set(v8::String::New(name), v8::Number::New(value));
    return scope.success();
}

bool ScriptObject::set(const char* name, unsigned long value)
{
    ScriptScope scope(m_scriptState);
    v8Object()->Set(v8::String::New(name), v8::Number::New(value));
    return scope.success();
}

bool ScriptObject::set(const char* name, bool value)
{
    ScriptScope scope(m_scriptState);
    v8Object()->Set(v8::String::New(name), v8Boolean(value));
    return scope.success();
}

ScriptObject ScriptObject::createNew(ScriptState* scriptState)
{
    ScriptScope scope(scriptState);
    return ScriptObject(scriptState, v8::Object::New());
}

bool ScriptGlobalObject::set(ScriptState* scriptState, const char* name, const ScriptObject& value)
{
    ScriptScope scope(scriptState);
    scope.global()->Set(v8::String::New(name), value.v8Value());
    return scope.success();
}

#if ENABLE(INSPECTOR)
bool ScriptGlobalObject::set(ScriptState* scriptState, const char* name, InspectorBackend* value)
{
    ScriptScope scope(scriptState);
    scope.global()->Set(v8::String::New(name), toV8(value));
    return scope.success();
}

bool ScriptGlobalObject::set(ScriptState* scriptState, const char* name, InspectorFrontendHost* value)
{
    ScriptScope scope(scriptState);
    scope.global()->Set(v8::String::New(name), toV8(value));
    return scope.success();
}

bool ScriptGlobalObject::set(ScriptState* scriptState, const char* name, InjectedScriptHost* value)
{
    ScriptScope scope(scriptState);
    scope.global()->Set(v8::String::New(name), toV8(value));
    return scope.success();
}
#endif

bool ScriptGlobalObject::get(ScriptState* scriptState, const char* name, ScriptObject& value)
{
    ScriptScope scope(scriptState);
    v8::Local<v8::Value> v8Value = scope.global()->Get(v8::String::New(name));
    if (v8Value.IsEmpty())
        return false;

    if (!v8Value->IsObject())
        return false;

    value = ScriptObject(scriptState, v8::Handle<v8::Object>(v8::Object::Cast(*v8Value)));
    return true;
}

bool ScriptGlobalObject::remove(ScriptState* scriptState, const char* name)
{
    ScriptScope scope(scriptState);
    return scope.global()->Delete(v8::String::New(name));
}

} // namespace WebCore
