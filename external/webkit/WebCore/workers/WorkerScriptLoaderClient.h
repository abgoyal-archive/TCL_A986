

#ifndef WorkerScriptLoaderClient_h
#define WorkerScriptLoaderClient_h

#if ENABLE(WORKERS)

namespace WebCore {

    class WorkerScriptLoaderClient {
    public:
        // FIXME: notifyFinished() is not currently guaranteed to be invoked if used from worker context and the worker shuts down in the middle of an operation.
        // This will cause leaks when we support nested workers.
        virtual void notifyFinished() { }

    protected:
        virtual ~WorkerScriptLoaderClient() { }
    };

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // WorkerScriptLoaderClient_h
