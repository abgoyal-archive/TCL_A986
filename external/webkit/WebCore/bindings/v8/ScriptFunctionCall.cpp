

#include "config.h"
#include "ScriptFunctionCall.h"

#include "ScriptScope.h"
#include "ScriptState.h"
#include "ScriptString.h"
#include "ScriptValue.h"

#include "V8Binding.h"
#include "V8Proxy.h"
#include "V8Utilities.h"

#include <v8.h>
#include <wtf/OwnArrayPtr.h>

namespace WebCore {

ScriptFunctionCall::ScriptFunctionCall(const ScriptObject& thisObject, const String& name)
    : m_scriptState(thisObject.scriptState())
    , m_thisObject(thisObject)
    , m_name(name)
{
}

void ScriptFunctionCall::appendArgument(const ScriptObject& argument)
{
    if (argument.scriptState() != m_scriptState) {
        ASSERT_NOT_REACHED();
        return;
    }
    m_arguments.append(argument);
}

void ScriptFunctionCall::appendArgument(const ScriptString& argument)
{
    ScriptScope scope(m_scriptState);
    m_arguments.append(v8String(argument));
}

void ScriptFunctionCall::appendArgument(const ScriptValue& argument)
{
    m_arguments.append(argument);
}

void ScriptFunctionCall::appendArgument(const String& argument)
{
    ScriptScope scope(m_scriptState);
    m_arguments.append(v8String(argument));
}

void ScriptFunctionCall::appendArgument(const char* argument)
{
    ScriptScope scope(m_scriptState);
    m_arguments.append(v8String(argument));
}

void ScriptFunctionCall::appendArgument(long argument)
{
    ScriptScope scope(m_scriptState);
    m_arguments.append(v8::Number::New(argument));
}

void ScriptFunctionCall::appendArgument(long long argument)
{
    ScriptScope scope(m_scriptState);
    m_arguments.append(v8::Number::New(argument));
}

void ScriptFunctionCall::appendArgument(unsigned int argument)
{
    ScriptScope scope(m_scriptState);
    m_arguments.append(v8::Number::New(argument));
}

void ScriptFunctionCall::appendArgument(unsigned long argument)
{
    ScriptScope scope(m_scriptState);
    m_arguments.append(v8::Number::New(argument));
}

void ScriptFunctionCall::appendArgument(int argument)
{
    ScriptScope scope(m_scriptState);
    m_arguments.append(v8::Number::New(argument));
}

void ScriptFunctionCall::appendArgument(bool argument)
{
    m_arguments.append(v8Boolean(argument));
}

ScriptValue ScriptFunctionCall::call(bool& hadException, bool reportExceptions)
{
    ScriptScope scope(m_scriptState, reportExceptions);

    v8::Local<v8::Object> thisObject = m_thisObject.v8Object();
    v8::Local<v8::Value> value = thisObject->Get(v8String(m_name));
    if (!scope.success()) {
        hadException = true;
        return ScriptValue();
    }

    ASSERT(value->IsFunction());

    v8::Local<v8::Function> function(v8::Function::Cast(*value));
    OwnArrayPtr<v8::Handle<v8::Value> > args(new v8::Handle<v8::Value>[m_arguments.size()]);
    for (size_t i = 0; i < m_arguments.size(); ++i)
        args[i] = m_arguments[i].v8Value();

    v8::Local<v8::Value> result = function->Call(thisObject, m_arguments.size(), args.get());
    if (!scope.success()) {
        hadException = true;
        return ScriptValue();
    }

    return ScriptValue(result);
}

ScriptValue ScriptFunctionCall::call()
{
    bool hadException = false;
    return call(hadException);
}

ScriptObject ScriptFunctionCall::construct(bool& hadException, bool reportExceptions)
{
    ScriptScope scope(m_scriptState, reportExceptions);

    v8::Local<v8::Object> thisObject = m_thisObject.v8Object();
    v8::Local<v8::Value> value = thisObject->Get(v8String(m_name));
    if (!scope.success()) {
        hadException = true;
        return ScriptObject();
    }

    ASSERT(value->IsFunction());

    v8::Local<v8::Function> constructor(v8::Function::Cast(*value));
    OwnArrayPtr<v8::Handle<v8::Value> > args(new v8::Handle<v8::Value>[m_arguments.size()]);
    for (size_t i = 0; i < m_arguments.size(); ++i)
        args[i] = m_arguments[i].v8Value();

    v8::Local<v8::Object> result = SafeAllocation::newInstance(constructor, m_arguments.size(), args.get());
    if (!scope.success()) {
        hadException = true;
        return ScriptObject();
    }

    return ScriptObject(m_scriptState, result);
}

} // namespace WebCore
