

#ifndef WorkerReportingProxy_h
#define WorkerReportingProxy_h

#if ENABLE(WORKERS)

#include "Console.h"

namespace WebCore {

    class String;

    // APIs used by workers to report console activity.
    class WorkerReportingProxy {
    public:
        virtual ~WorkerReportingProxy() {}

        virtual void postExceptionToWorkerObject(const String& errorMessage, int lineNumber, const String& sourceURL) = 0;

        virtual void postConsoleMessageToWorkerObject(MessageDestination, MessageSource, MessageType, MessageLevel, const String& message, int lineNumber, const String& sourceURL) = 0;

        // Invoked when close() is invoked on the worker context.
        virtual void workerContextClosed() = 0;

        // Invoked when the thread has stopped.
        virtual void workerContextDestroyed() = 0;
    };
} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // WorkerReportingProxy_h
