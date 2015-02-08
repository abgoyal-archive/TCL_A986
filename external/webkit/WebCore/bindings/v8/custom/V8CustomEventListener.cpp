

#include "config.h"
#include "V8CustomEventListener.h"

#include "V8Proxy.h"

namespace WebCore {

V8EventListener::V8EventListener(v8::Local<v8::Object> listener, bool isAttribute, const WorldContextHandle& worldContext)
    : V8AbstractEventListener(isAttribute, worldContext)
{
    setListenerObject(listener);
}

v8::Local<v8::Function> V8EventListener::getListenerFunction(ScriptExecutionContext* context)
{
    v8::Local<v8::Object> listener = getListenerObject(context);

    // Has the listener been disposed?
    if (listener.IsEmpty())
        return v8::Local<v8::Function>();

    if (listener->IsFunction())
        return v8::Local<v8::Function>::Cast(listener);

    if (listener->IsObject()) {
        v8::Local<v8::Value> property = listener->Get(v8::String::NewSymbol("handleEvent"));
        if (property->IsFunction())
            return v8::Local<v8::Function>::Cast(property);
    }

    return v8::Local<v8::Function>();
}

v8::Local<v8::Value> V8EventListener::callListenerFunction(ScriptExecutionContext* context, v8::Handle<v8::Value> jsEvent, Event* event)
{

    v8::Local<v8::Function> handlerFunction = getListenerFunction(context);
    v8::Local<v8::Object> receiver = getReceiverObject(event);
    if (handlerFunction.IsEmpty() || receiver.IsEmpty())
        return v8::Local<v8::Value>();

    v8::Handle<v8::Value> parameters[1] = { jsEvent };

    if (V8Proxy* proxy = V8Proxy::retrieve(context))
        return proxy->callFunction(handlerFunction, receiver, 1, parameters);

    return v8::Local<v8::Value>();
}

} // namespace WebCore
