

#include "config.h"
#include "ScriptArray.h"

#include "ScriptScope.h"
#include "ScriptState.h"

#include "Document.h"
#include "Frame.h"
#include "V8Binding.h"
#include "V8Proxy.h"

#include <v8.h>

namespace WebCore {

ScriptArray::ScriptArray(ScriptState* scriptState, v8::Handle<v8::Array> v8Array)
    : ScriptObject(scriptState, v8Array)
{
}

bool ScriptArray::set(unsigned index, const ScriptObject& value)
{
    if (value.scriptState() != m_scriptState) {
        ASSERT_NOT_REACHED();
        return false;
    }
    ScriptScope scope(m_scriptState);
    v8Object()->Set(v8::Integer::New(index), value.v8Value());
    return scope.success();
}

bool ScriptArray::set(unsigned index, const String& value)
{
    ScriptScope scope(m_scriptState);
    v8Object()->Set(v8::Integer::New(index), v8String(value));
    return scope.success();
}

bool ScriptArray::set(unsigned index, double value)
{
    ScriptScope scope(m_scriptState);
    v8Object()->Set(v8::Integer::New(index), v8::Number::New(value));
    return scope.success();
}

bool ScriptArray::set(unsigned index, long long value)
{
    ScriptScope scope(m_scriptState);
    v8Object()->Set(v8::Integer::New(index), v8::Number::New(value));
    return scope.success();
}

bool ScriptArray::set(unsigned index, int value)
{
    ScriptScope scope(m_scriptState);
    v8Object()->Set(v8::Integer::New(index), v8::Number::New(value));
    return scope.success();
}

bool ScriptArray::set(unsigned index, bool value)
{
    ScriptScope scope(m_scriptState);
    v8Object()->Set(v8::Integer::New(index), v8Boolean(value));
    return scope.success();
}

unsigned ScriptArray::length()
{
    ScriptScope scope(m_scriptState);
    return v8::Array::Cast(*v8Value())->Length();
}

ScriptArray ScriptArray::createNew(ScriptState* scriptState)
{
    ScriptScope scope(scriptState);
    return ScriptArray(scriptState, v8::Array::New());
}

} // namespace WebCore
