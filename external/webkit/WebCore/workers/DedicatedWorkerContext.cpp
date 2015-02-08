

#include "config.h"

#if ENABLE(WORKERS)

#include "DedicatedWorkerContext.h"

#include "DedicatedWorkerThread.h"
#include "DOMWindow.h"
#include "MessageEvent.h"
#include "WorkerObjectProxy.h"

namespace WebCore {

DedicatedWorkerContext::DedicatedWorkerContext(const KURL& url, const String& userAgent, DedicatedWorkerThread* thread)
    : WorkerContext(url, userAgent, thread)
{
}

// FIXME: remove this when we update the ObjC bindings (bug #28774).
void DedicatedWorkerContext::postMessage(PassRefPtr<SerializedScriptValue> message, MessagePort* port, ExceptionCode& ec)
{
    MessagePortArray ports;
    if (port)
        ports.append(port);
    postMessage(message, &ports, ec);
}

void DedicatedWorkerContext::postMessage(PassRefPtr<SerializedScriptValue> message, ExceptionCode& ec)
{
    postMessage(message, static_cast<MessagePortArray*>(0), ec);
}

void DedicatedWorkerContext::postMessage(PassRefPtr<SerializedScriptValue> message, const MessagePortArray* ports, ExceptionCode& ec)
{
    if (isClosing())
        return;
    // Disentangle the port in preparation for sending it to the remote context.
    OwnPtr<MessagePortChannelArray> channels = MessagePort::disentanglePorts(ports, ec);
    if (ec)
        return;
    thread()->workerObjectProxy().postMessageToWorkerObject(message, channels.release());
}

void DedicatedWorkerContext::importScripts(const Vector<String>& urls, const String& callerURL, int callerLine, ExceptionCode& ec)
{
    Base::importScripts(urls, callerURL, callerLine, ec);
    thread()->workerObjectProxy().reportPendingActivity(hasPendingActivity());
}

DedicatedWorkerThread* DedicatedWorkerContext::thread()
{
    return static_cast<DedicatedWorkerThread*>(Base::thread());
}

} // namespace WebCore

#endif // ENABLE(WORKERS)
