

#include "config.h"
#include "ThreadableLoader.h"

#include "ScriptExecutionContext.h"
#include "Document.h"
#include "DocumentThreadableLoader.h"
#include "WorkerContext.h"
#include "WorkerRunLoop.h"
#include "WorkerThreadableLoader.h"

namespace WebCore {

PassRefPtr<ThreadableLoader> ThreadableLoader::create(ScriptExecutionContext* context, ThreadableLoaderClient* client, const ResourceRequest& request, const ThreadableLoaderOptions& options) 
{
    ASSERT(client);
    ASSERT(context);

#if ENABLE(WORKERS)
    if (context->isWorkerContext())
        return WorkerThreadableLoader::create(static_cast<WorkerContext*>(context), client, WorkerRunLoop::defaultMode(), request, options);
#endif // ENABLE(WORKERS)

    ASSERT(context->isDocument());
    return DocumentThreadableLoader::create(static_cast<Document*>(context), client, request, options);
}

void ThreadableLoader::loadResourceSynchronously(ScriptExecutionContext* context, const ResourceRequest& request, ThreadableLoaderClient& client, const ThreadableLoaderOptions& options)
{
    ASSERT(context);

#if ENABLE(WORKERS)
    if (context->isWorkerContext()) {
        WorkerThreadableLoader::loadResourceSynchronously(static_cast<WorkerContext*>(context), request, client, options);
        return;
    }
#endif // ENABLE(WORKERS)

    ASSERT(context->isDocument());
    DocumentThreadableLoader::loadResourceSynchronously(static_cast<Document*>(context), request, client, options);
}

} // namespace WebCore
