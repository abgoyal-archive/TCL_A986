

#include "config.h"
#include "V8PopStateEvent.h"

#include "PopStateEvent.h"
#include "SerializedScriptValue.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8PopStateEvent::initPopStateEventCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.PopStateEvent.initPopStateEvent");

    String typeArg = v8ValueToWebCoreString(args[0]);
    bool canBubbleArg = args[1]->BooleanValue();
    bool cancelableArg = args[2]->BooleanValue();
    RefPtr<SerializedScriptValue> stateArg = SerializedScriptValue::create(args[3]);

    PopStateEvent* event = V8PopStateEvent::toNative(args.Holder());
    event->initPopStateEvent(typeArg, canBubbleArg, cancelableArg, stateArg.release());

    return v8::Undefined();
}

v8::Handle<v8::Value> V8PopStateEvent::stateAccessorGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.PopStateEvent.state");

    PopStateEvent* event = V8PopStateEvent::toNative(info.Holder());
    SerializedScriptValue* state = event->state();
    if (!state)
        return v8::Null();

    return state->deserialize();
}

} // namespace WebCore
