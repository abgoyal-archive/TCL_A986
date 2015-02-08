

#include "config.h"
#include "WebSharedWorkerImpl.h"

#include "GenericWorkerTask.h"
#include "KURL.h"
#include "MessageEvent.h"
#include "MessagePortChannel.h"
#include "PlatformMessagePortChannel.h"
#include "ScriptExecutionContext.h"
#include "SharedWorkerContext.h"
#include "SharedWorkerThread.h"

#include "WebMessagePortChannel.h"
#include "WebString.h"
#include "WebURL.h"

using namespace WebCore;

namespace WebKit {

#if ENABLE(SHARED_WORKERS)

WebSharedWorkerImpl::WebSharedWorkerImpl(WebCommonWorkerClient* client)
    : m_client(client)
{
}

WebSharedWorkerImpl::~WebSharedWorkerImpl()
{
}

bool WebSharedWorkerImpl::isStarted()
{
    // Should not ever be called from the worker thread (this API is only called on WebSharedWorkerProxy on the renderer thread).
    ASSERT_NOT_REACHED();
    return workerThread();
}

void WebSharedWorkerImpl::connect(WebMessagePortChannel* webChannel, ConnectListener* listener)
{
    // Convert the WebMessagePortChanel to a WebCore::MessagePortChannel.
    RefPtr<PlatformMessagePortChannel> platform_channel =
        PlatformMessagePortChannel::create(webChannel);
    webChannel->setClient(platform_channel.get());
    OwnPtr<MessagePortChannel> channel =
        MessagePortChannel::create(platform_channel);

    workerThread()->runLoop().postTask(
        createCallbackTask(&connectTask, this, channel.release()));
    if (listener)
        listener->connected();
}

void WebSharedWorkerImpl::connectTask(ScriptExecutionContext* context, WebSharedWorkerImpl* worker, PassOwnPtr<MessagePortChannel> channel)
{
    // Wrap the passed-in channel in a MessagePort, and send it off via a connect event.
    RefPtr<MessagePort> port = MessagePort::create(*context);
    port->entangle(channel.release());
    ASSERT(context->isWorkerContext());
    WorkerContext* workerContext = static_cast<WorkerContext*>(context);
    ASSERT(workerContext->isSharedWorkerContext());
    workerContext->toSharedWorkerContext()->dispatchEvent(createConnectEvent(port));
}

void WebSharedWorkerImpl::startWorkerContext(const WebURL& url, const WebString& name, const WebString& userAgent, const WebString& sourceCode)
{
    initializeLoader(url);
    setWorkerThread(SharedWorkerThread::create(name, url, userAgent, sourceCode, *this, *this));
    workerThread()->start();
}

void WebSharedWorkerImpl::terminateWorkerContext()
{
    stopWorkerThread();
}

void WebSharedWorkerImpl::clientDestroyed()
{
    m_client = 0;
}

WebWorkerClient* WebSharedWorkerImpl::client()
{
    // We should never be asked for a WebWorkerClient (only dedicated workers have an associated WebWorkerClient).
    // It should not be possible for SharedWorkerContext to generate an API call outside those supported by WebCommonWorkerClient.
    ASSERT_NOT_REACHED();
    return 0;
}

WebSharedWorker* WebSharedWorker::create(WebCommonWorkerClient* client)
{
    return new WebSharedWorkerImpl(client);
}

#endif // ENABLE(SHARED_WORKERS)

} // namespace WebKit
