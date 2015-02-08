

#include "config.h"

#if ENABLE(WORKERS)

#include "Worker.h"

#include "DOMWindow.h"
#include "DocLoader.h"
#include "Document.h"
#include "EventException.h"
#include "EventListener.h"
#include "EventNames.h"
#include "ExceptionCode.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "MessageEvent.h"
#include "TextEncoding.h"
#include "WorkerContextProxy.h"
#include "WorkerScriptLoader.h"
#include "WorkerThread.h"
#include <wtf/MainThread.h>

namespace WebCore {

Worker::Worker(const String& url, ScriptExecutionContext* context, ExceptionCode& ec)
    : AbstractWorker(context)
    , m_contextProxy(WorkerContextProxy::create(this))
{
    KURL scriptURL = resolveURL(url, ec);
    if (ec)
        return;

    m_scriptLoader = new WorkerScriptLoader();
    m_scriptLoader->loadAsynchronously(scriptExecutionContext(), scriptURL, DenyCrossOriginRequests, this);
    setPendingActivity(this);  // The worker context does not exist while loading, so we must ensure that the worker object is not collected, as well as its event listeners.
}

Worker::~Worker()
{
    ASSERT(isMainThread());
    ASSERT(scriptExecutionContext()); // The context is protected by worker context proxy, so it cannot be destroyed while a Worker exists.
    m_contextProxy->workerObjectDestroyed();
}

// FIXME: remove this when we update the ObjC bindings (bug #28774).
void Worker::postMessage(PassRefPtr<SerializedScriptValue> message, MessagePort* port, ExceptionCode& ec)
{
    MessagePortArray ports;
    if (port)
        ports.append(port);
    postMessage(message, &ports, ec);
}

void Worker::postMessage(PassRefPtr<SerializedScriptValue> message, ExceptionCode& ec)
{
    postMessage(message, static_cast<MessagePortArray*>(0), ec);
}

void Worker::postMessage(PassRefPtr<SerializedScriptValue> message, const MessagePortArray* ports, ExceptionCode& ec)
{
    // Disentangle the port in preparation for sending it to the remote context.
    OwnPtr<MessagePortChannelArray> channels = MessagePort::disentanglePorts(ports, ec);
    if (ec)
        return;
    m_contextProxy->postMessageToWorkerContext(message, channels.release());
}

void Worker::terminate()
{
    m_contextProxy->terminateWorkerContext();
}

bool Worker::canSuspend() const
{
    // FIXME: It is not currently possible to suspend a worker, so pages with workers can not go into page cache.
    return false;
}

void Worker::stop()
{
    terminate();
}

bool Worker::hasPendingActivity() const
{
    return m_contextProxy->hasPendingActivity() || ActiveDOMObject::hasPendingActivity();
}

void Worker::notifyFinished()
{
    if (m_scriptLoader->failed())
        dispatchEvent(Event::create(eventNames().errorEvent, false, true));
    else
        m_contextProxy->startWorkerContext(m_scriptLoader->url(), scriptExecutionContext()->userAgent(m_scriptLoader->url()), m_scriptLoader->script());

    m_scriptLoader = 0;

    unsetPendingActivity(this);
}

} // namespace WebCore

#endif // ENABLE(WORKERS)
