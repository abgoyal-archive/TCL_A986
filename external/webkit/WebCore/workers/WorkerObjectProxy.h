

#ifndef WorkerObjectProxy_h
#define WorkerObjectProxy_h

#if ENABLE(WORKERS)

#include "WorkerReportingProxy.h"
#include "MessagePort.h"
#include <wtf/PassOwnPtr.h>

namespace WebCore {

    class MessagePortChannel;

    // A proxy to talk to the worker object.
    class WorkerObjectProxy : public WorkerReportingProxy {
    public:
        virtual void postMessageToWorkerObject(PassRefPtr<SerializedScriptValue>, PassOwnPtr<MessagePortChannelArray>) = 0;

        virtual void confirmMessageFromWorkerObject(bool hasPendingActivity) = 0;
        virtual void reportPendingActivity(bool hasPendingActivity) = 0;

        // No need to notify the parent page context when dedicated workers are closing.
        virtual void workerContextClosed() { }
    };

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // WorkerObjectProxy_h
