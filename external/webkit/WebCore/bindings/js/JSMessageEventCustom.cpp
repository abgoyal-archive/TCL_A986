

#include "config.h"
#include "JSMessageEvent.h"

#include "JSDOMBinding.h"
#include "JSDOMWindow.h"
#include "JSEventTarget.h"
#include "JSMessagePortCustom.h"
#include "MessageEvent.h"
#include <runtime/JSArray.h>

using namespace JSC;

namespace WebCore {

JSValue JSMessageEvent::ports(ExecState* exec) const
{
    MessagePortArray* ports = static_cast<MessageEvent*>(impl())->ports();
    if (!ports || ports->isEmpty())
        return jsNull();

    MarkedArgumentBuffer list;
    for (size_t i = 0; i < ports->size(); i++)
        list.append(toJS(exec, globalObject(), (*ports)[i].get()));
    return constructArray(exec, list);
}

JSC::JSValue JSMessageEvent::initMessageEvent(JSC::ExecState* exec, const JSC::ArgList& args)
{
    const UString& typeArg = args.at(0).toString(exec);
    bool canBubbleArg = args.at(1).toBoolean(exec);
    bool cancelableArg = args.at(2).toBoolean(exec);
    PassRefPtr<SerializedScriptValue> dataArg = SerializedScriptValue::create(exec, args.at(3));
    const UString& originArg = args.at(4).toString(exec);
    const UString& lastEventIdArg = args.at(5).toString(exec);
    DOMWindow* sourceArg = toDOMWindow(args.at(6));
    OwnPtr<MessagePortArray> messagePorts;
    if (!args.at(7).isUndefinedOrNull()) {
        messagePorts = new MessagePortArray();
        fillMessagePortArray(exec, args.at(7), *messagePorts);
        if (exec->hadException())
            return jsUndefined();
    }

    MessageEvent* event = static_cast<MessageEvent*>(this->impl());
    event->initMessageEvent(typeArg, canBubbleArg, cancelableArg, dataArg, originArg, lastEventIdArg, sourceArg, messagePorts.release());
    return jsUndefined();
}

} // namespace WebCore
