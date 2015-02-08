

#include "config.h"
#include "V8NodeFilterCondition.h"

#include "Node.h"
#include "NodeFilter.h"
#include "ScriptState.h"
#include "V8Node.h"
#include "V8Proxy.h"

#include <wtf/OwnArrayPtr.h>

namespace WebCore {

V8NodeFilterCondition::V8NodeFilterCondition(v8::Handle<v8::Value> filter)
    : m_filter(v8::Persistent<v8::Value>::New(filter))
{
#ifndef NDEBUG
    V8GCController::registerGlobalHandle(NODE_FILTER, this, m_filter);
#endif
}

V8NodeFilterCondition::~V8NodeFilterCondition()
{
#ifndef NDEBUG
    V8GCController::unregisterGlobalHandle(this, m_filter);
#endif
    m_filter.Dispose();
    m_filter.Clear();
}

short V8NodeFilterCondition::acceptNode(ScriptState* state, Node* node) const
{
    ASSERT(v8::Context::InContext());

    if (!m_filter->IsFunction())
        return NodeFilter::FILTER_ACCEPT;

    v8::TryCatch exceptionCatcher;

    v8::Handle<v8::Object> object = v8::Context::GetCurrent()->Global();
    v8::Handle<v8::Function> callback = v8::Handle<v8::Function>::Cast(m_filter);
    OwnArrayPtr<v8::Handle<v8::Value> > args(new v8::Handle<v8::Value>[1]);
    args[0] = toV8(node);

    V8Proxy* proxy = V8Proxy::retrieve();
    ASSERT(proxy);

    v8::Handle<v8::Value> result = proxy->callFunction(callback, object, 1, args.get());

    if (exceptionCatcher.HasCaught()) {
        state->setException(exceptionCatcher.Exception());
        return NodeFilter::FILTER_REJECT;
    }

    ASSERT(!result.IsEmpty());

    return result->Int32Value();
}

} // namespace WebCore
