

#include "config.h"
#include "JSPopStateEvent.h"

#include "PopStateEvent.h"

using namespace JSC;

namespace WebCore {

JSValue JSPopStateEvent::initPopStateEvent(ExecState* exec, const ArgList& args)
{
    const UString& typeArg = args.at(0).toString(exec);
    bool canBubbleArg = args.at(1).toBoolean(exec);
    bool cancelableArg = args.at(2).toBoolean(exec);
    RefPtr<SerializedScriptValue> stateObjectArg = SerializedScriptValue::create(exec, args.at(3));
    
    PopStateEvent* event = static_cast<PopStateEvent*>(impl());
    event->initPopStateEvent(typeArg, canBubbleArg, cancelableArg, stateObjectArg.release());
    return jsUndefined();
}

} // namespace WebCore
