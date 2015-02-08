

#ifndef WorkerLoaderProxy_h
#define WorkerLoaderProxy_h

#if ENABLE(WORKERS)

#include "ScriptExecutionContext.h"
#include <wtf/PassOwnPtr.h>

namespace WebCore {

    class String;

    // A proxy to talk to the loader context. Normally, the document on the main thread
    // provides loading services for the subordinate workers. This interface provides 2-way
    // communications to the Document context and back to the worker.
    // Note that in multi-process browsers, the Worker object context and the Document
    // context can be distinct.
    class WorkerLoaderProxy {
    public:
        virtual ~WorkerLoaderProxy() { }

        // Posts a task to the thread which runs the loading code (normally, the main thread).
        virtual void postTaskToLoader(PassOwnPtr<ScriptExecutionContext::Task>) = 0;

        // Posts callbacks from loading code to the WorkerContext. The 'mode' is used to differentiate
        // specific synchronous loading requests so they can be 'nested', per spec.
        virtual void postTaskForModeToWorkerContext(PassOwnPtr<ScriptExecutionContext::Task>, const String& mode) = 0;
    };

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // WorkerLoaderProxy_h
