

#ifndef WorkerThreadableLoader_h
#define WorkerThreadableLoader_h

#if ENABLE(WORKERS)

#include "PlatformString.h"
#include "ThreadableLoader.h"
#include "ThreadableLoaderClient.h"
#include "ThreadableLoaderClientWrapper.h"

#include <memory>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>
#include <wtf/Threading.h>

namespace WebCore {

    class ResourceError;
    class ResourceRequest;
    class WorkerContext;
    class WorkerLoaderProxy;
    struct CrossThreadResourceResponseData;
    struct CrossThreadResourceRequestData;

    class WorkerThreadableLoader : public RefCounted<WorkerThreadableLoader>, public ThreadableLoader {
    public:
        static void loadResourceSynchronously(WorkerContext*, const ResourceRequest&, ThreadableLoaderClient&, const ThreadableLoaderOptions&);
        static PassRefPtr<WorkerThreadableLoader> create(WorkerContext* workerContext, ThreadableLoaderClient* client, const String& taskMode, const ResourceRequest& request, const ThreadableLoaderOptions& options)
        {
            return adoptRef(new WorkerThreadableLoader(workerContext, client, taskMode, request, options));
        }

        ~WorkerThreadableLoader();

        virtual void cancel();

        bool done() const { return m_workerClientWrapper->done(); }

        using RefCounted<WorkerThreadableLoader>::ref;
        using RefCounted<WorkerThreadableLoader>::deref;

    protected:
        virtual void refThreadableLoader() { ref(); }
        virtual void derefThreadableLoader() { deref(); }

    private:
        // Creates a loader on the main thread and bridges communication between
        // the main thread and the worker context's thread where WorkerThreadableLoader runs.
        //
        // Regarding the bridge and lifetimes of items used in callbacks, there are a few cases:
        //
        // all cases. All tasks posted from the worker context's thread are ok because
        //    the last task posted always is "mainThreadDestroy", so MainThreadBridge is
        //    around for all tasks that use it on the main thread.
        //
        // case 1. worker.terminate is called.
        //    In this case, no more tasks are posted from the worker object's thread to the worker
        //    context's thread -- WorkerContextProxy implementation enforces this.
        //
        // case 2. xhr gets aborted and the worker context continues running.
        //    The ThreadableLoaderClientWrapper has the underlying client cleared, so no more calls
        //    go through it.  All tasks posted from the worker object's thread to the worker context's
        //    thread do "ThreadableLoaderClientWrapper::ref" (automatically inside of the cross thread copy
        //    done in createCallbackTask), so the ThreadableLoaderClientWrapper instance is there until all
        //    tasks are executed.
        class MainThreadBridge : public ThreadableLoaderClient {
        public:
            // All executed on the worker context's thread.
            MainThreadBridge(PassRefPtr<ThreadableLoaderClientWrapper>, WorkerLoaderProxy&, const String& taskMode, const ResourceRequest&, const ThreadableLoaderOptions&);
            void cancel();
            void destroy();

        private:
            // Executed on the worker context's thread.
            void clearClientWrapper();

            // All executed on the main thread.
            static void mainThreadDestroy(ScriptExecutionContext*, MainThreadBridge*);
            ~MainThreadBridge();

            static void mainThreadCreateLoader(ScriptExecutionContext*, MainThreadBridge*, std::auto_ptr<CrossThreadResourceRequestData>, ThreadableLoaderOptions);
            static void mainThreadCancel(ScriptExecutionContext*, MainThreadBridge*);
            virtual void didSendData(unsigned long long bytesSent, unsigned long long totalBytesToBeSent);
            virtual void didReceiveResponse(const ResourceResponse&);
            virtual void didReceiveData(const char*, int lengthReceived);
            virtual void didFinishLoading(unsigned long identifier);
            virtual void didFail(const ResourceError&);
            virtual void didFailRedirectCheck();
            virtual void didReceiveAuthenticationCancellation(const ResourceResponse&);

            // Only to be used on the main thread.
            RefPtr<ThreadableLoader> m_mainThreadLoader;

            // ThreadableLoaderClientWrapper is to be used on the worker context thread.
            // The ref counting is done on either thread.
            RefPtr<ThreadableLoaderClientWrapper> m_workerClientWrapper;

            // May be used on either thread.
            WorkerLoaderProxy& m_loaderProxy;

            // For use on the main thread.
            String m_taskMode;
        };

        WorkerThreadableLoader(WorkerContext*, ThreadableLoaderClient*, const String& taskMode, const ResourceRequest&, const ThreadableLoaderOptions&);

        RefPtr<WorkerContext> m_workerContext;
        RefPtr<ThreadableLoaderClientWrapper> m_workerClientWrapper;
        MainThreadBridge& m_bridge;
    };

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // WorkerThreadableLoader_h
