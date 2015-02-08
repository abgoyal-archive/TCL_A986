

#include "config.h"
#include "JSMessagePort.h"

#include "AtomicString.h"
#include "Event.h"
#include "ExceptionCode.h"
#include "Frame.h"
#include "JSDOMGlobalObject.h"
#include "JSEvent.h"
#include "JSEventListener.h"
#include "JSMessagePortCustom.h"
#include "MessagePort.h"
#include <runtime/Error.h>

using namespace JSC;

namespace WebCore {

void JSMessagePort::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    // If we have a locally entangled port, we can directly mark it as reachable. Ports that are remotely entangled are marked in-use by markActiveObjectsForContext().
    if (MessagePort* entangledPort = m_impl->locallyEntangledPort())
        markDOMObjectWrapper(markStack, *Heap::heap(this)->globalData(), entangledPort);

    m_impl->markJSEventListeners(markStack);
}

JSValue JSMessagePort::addEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->addEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)).get(), args.at(2).toBoolean(exec));
    return jsUndefined();
}

JSValue JSMessagePort::removeEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->removeEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)).get(), args.at(2).toBoolean(exec));
    return jsUndefined();
}

JSC::JSValue JSMessagePort::postMessage(JSC::ExecState* exec, const JSC::ArgList& args)
{
    return handlePostMessage(exec, args, impl());
}

void fillMessagePortArray(JSC::ExecState* exec, JSC::JSValue value, MessagePortArray& portArray)
{
    // Convert from the passed-in JS array-like object to a MessagePortArray.
    // Also validates the elements per sections 4.1.13 and 4.1.15 of the WebIDL spec and section 8.3.3 of the HTML5 spec.
    if (value.isUndefinedOrNull()) {
        portArray.resize(0);
        return;
    }

    // Validation of sequence types, per WebIDL spec 4.1.13.
    unsigned length;
    JSObject* object = toJSSequence(exec, value, length);
    if (exec->hadException())
        return;

    portArray.resize(length);
    for (unsigned i = 0 ; i < length; ++i) {
        JSValue value = object->get(exec, i);
        if (exec->hadException())
            return;
        // Validation of non-null objects, per HTML5 spec 8.3.3.
        if (value.isUndefinedOrNull()) {
            setDOMException(exec, INVALID_STATE_ERR);
            return;
        }

        // Validation of Objects implementing an interface, per WebIDL spec 4.1.15.
        RefPtr<MessagePort> port = toMessagePort(value);
        if (!port) {
            throwError(exec, TypeError);
            return;
        }
        portArray[i] = port.release();
    }
}

} // namespace WebCore
