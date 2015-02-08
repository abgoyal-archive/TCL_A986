

#include "config.h"

#if ENABLE(EVENTSOURCE)

#include "JSEventSource.h"

#include "EventSource.h"
#include "JSDOMGlobalObject.h"
#include "JSEventListener.h"

using namespace JSC;

namespace WebCore {

JSValue JSEventSource::addEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->addEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)).get(), args.at(2).toBoolean(exec));
    return jsUndefined();
}

JSValue JSEventSource::removeEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->removeEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)).get(), args.at(2).toBoolean(exec));
    return jsUndefined();
}

} // namespace WebCore

#endif // ENABLE(EVENTSOURCE)
