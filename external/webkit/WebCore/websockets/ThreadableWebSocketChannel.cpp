

#include "config.h"

#if ENABLE(WEB_SOCKETS)

#include "ThreadableWebSocketChannel.h"

#include "PlatformString.h"
#include "ScriptExecutionContext.h"
#include "ThreadableWebSocketChannelClientWrapper.h"
#include "WebSocketChannel.h"
#include "WebSocketChannelClient.h"
#include "WorkerContext.h"
#include "WorkerRunLoop.h"
#include "WorkerThread.h"
#include "WorkerThreadableWebSocketChannel.h"

#include <wtf/PassRefPtr.h>

namespace WebCore {

static const char webSocketChannelMode[] = "webSocketChannelMode";

PassRefPtr<ThreadableWebSocketChannel> ThreadableWebSocketChannel::create(ScriptExecutionContext* context, WebSocketChannelClient* client, const KURL& url, const String& protocol)
{
    ASSERT(context);
    ASSERT(client);

#if ENABLE(WORKERS)
    if (context->isWorkerContext()) {
        WorkerContext* workerContext = static_cast<WorkerContext*>(context);
        WorkerRunLoop& runLoop = workerContext->thread()->runLoop();
        String mode = webSocketChannelMode;
        mode.append(String::number(runLoop.createUniqueId()));
        return WorkerThreadableWebSocketChannel::create(workerContext, client, mode, url, protocol);
    }
#endif // ENABLE(WORKERS)

    ASSERT(context->isDocument());
    return WebSocketChannel::create(context, client, url, protocol);
}

} // namespace WebCore

#endif // ENABLE(WEB_SOCKETS)
