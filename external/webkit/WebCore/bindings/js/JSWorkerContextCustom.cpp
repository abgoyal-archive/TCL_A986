

#include "config.h"

#if ENABLE(WORKERS)

#include "JSWorkerContext.h"

#include "JSDOMBinding.h"
#include "JSDOMGlobalObject.h"
#include "JSEventListener.h"
#include "JSEventSourceConstructor.h"
#include "JSMessageChannelConstructor.h"
#include "JSMessagePort.h"
#include "JSWebSocketConstructor.h"
#include "JSWorkerLocation.h"
#include "JSWorkerNavigator.h"
#include "JSXMLHttpRequestConstructor.h"
#include "ScheduledAction.h"
#include "WorkerContext.h"
#include "WorkerLocation.h"
#include "WorkerNavigator.h"
#include <interpreter/Interpreter.h>

using namespace JSC;

namespace WebCore {

void JSWorkerContext::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    JSGlobalData& globalData = *this->globalData();

    markActiveObjectsForContext(markStack, globalData, scriptExecutionContext());

    markDOMObjectWrapper(markStack, globalData, impl()->optionalLocation());
    markDOMObjectWrapper(markStack, globalData, impl()->optionalNavigator());

    impl()->markJSEventListeners(markStack);
}

bool JSWorkerContext::getOwnPropertySlotDelegate(ExecState* exec, const Identifier& propertyName, PropertySlot& slot)
{
    // Look for overrides before looking at any of our own properties.
    if (JSGlobalObject::getOwnPropertySlot(exec, propertyName, slot))
        return true;
    return false;
}

bool JSWorkerContext::getOwnPropertyDescriptorDelegate(ExecState* exec, const Identifier& propertyName, PropertyDescriptor& descriptor)
{
    // Look for overrides before looking at any of our own properties.
    if (JSGlobalObject::getOwnPropertyDescriptor(exec, propertyName, descriptor))
        return true;
    return false;
}

#if ENABLE(EVENTSOURCE)
JSValue JSWorkerContext::eventSource(ExecState* exec) const
{
    return getDOMConstructor<JSEventSourceConstructor>(exec, this);
}
#endif

JSValue JSWorkerContext::xmlHttpRequest(ExecState* exec) const
{
    return getDOMConstructor<JSXMLHttpRequestConstructor>(exec, this);
}

#if ENABLE(WEB_SOCKETS)
JSValue JSWorkerContext::webSocket(ExecState* exec) const
{
    return getDOMConstructor<JSWebSocketConstructor>(exec, this);
}
#endif

JSValue JSWorkerContext::importScripts(ExecState* exec, const ArgList& args)
{
    if (!args.size())
        return jsUndefined();

    Vector<String> urls;
    for (unsigned i = 0; i < args.size(); i++) {
        urls.append(args.at(i).toString(exec));
        if (exec->hadException())
            return jsUndefined();
    }
    ExceptionCode ec = 0;
    int signedLineNumber;
    intptr_t sourceID;
    UString sourceURL;
    JSValue function;
    exec->interpreter()->retrieveLastCaller(exec, signedLineNumber, sourceID, sourceURL, function);

    impl()->importScripts(urls, sourceURL, signedLineNumber >= 0 ? signedLineNumber : 0, ec);
    setDOMException(exec, ec);
    return jsUndefined();
}

JSValue JSWorkerContext::addEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->addEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)), args.at(2).toBoolean(exec));
    return jsUndefined();
}

JSValue JSWorkerContext::removeEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->removeEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)).get(), args.at(2).toBoolean(exec));
    return jsUndefined();
}

JSValue JSWorkerContext::setTimeout(ExecState* exec, const ArgList& args)
{
    ScheduledAction* action = ScheduledAction::create(exec, args, currentWorld(exec));
    if (exec->hadException())
        return jsUndefined();
    int delay = args.at(1).toInt32(exec);
    return jsNumber(exec, impl()->setTimeout(action, delay));
}

JSValue JSWorkerContext::setInterval(ExecState* exec, const ArgList& args)
{
    ScheduledAction* action = ScheduledAction::create(exec, args, currentWorld(exec));
    if (exec->hadException())
        return jsUndefined();
    int delay = args.at(1).toInt32(exec);
    return jsNumber(exec, impl()->setInterval(action, delay));
}


#if ENABLE(CHANNEL_MESSAGING)
JSValue JSWorkerContext::messageChannel(ExecState* exec) const
{
    return getDOMConstructor<JSMessageChannelConstructor>(exec, this);
}
#endif

} // namespace WebCore

#endif // ENABLE(WORKERS)
