

#ifndef DefaultSharedWorkerRepository_h
#define DefaultSharedWorkerRepository_h

#if ENABLE(SHARED_WORKERS)

#include "ExceptionCode.h"
#include "StringHash.h"
#include <wtf/HashMap.h>
#include <wtf/Noncopyable.h>
#include <wtf/PassOwnPtr.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>
#include <wtf/Threading.h>

namespace WebCore {

    class Document;
    class KURL;
    class MessagePortChannel;
    class ScriptExecutionContext;
    class SharedWorker;
    class SharedWorkerProxy;
    class String;

    // Platform-specific implementation of the SharedWorkerRepository static interface.
    class DefaultSharedWorkerRepository : public Noncopyable {
    public:
        // Invoked once the worker script has been loaded to fire up the worker thread.
        void workerScriptLoaded(SharedWorkerProxy&, const String& userAgent, const String& workerScript, PassOwnPtr<MessagePortChannel>);

        // Internal implementation of SharedWorkerRepository::connect()
        void connectToWorker(PassRefPtr<SharedWorker>, PassOwnPtr<MessagePortChannel>, const KURL&, const String& name, ExceptionCode&);

        // Notification that a document has been detached.
        void documentDetached(Document*);

        // Removes the passed SharedWorkerProxy from the repository.
        void removeProxy(SharedWorkerProxy*);

        bool hasSharedWorkers(Document*);

        static DefaultSharedWorkerRepository& instance();
    private:
        DefaultSharedWorkerRepository();
        ~DefaultSharedWorkerRepository();

        PassRefPtr<SharedWorkerProxy> getProxy(const String& name, const KURL&);
        // Mutex used to protect internal data structures.
        Mutex m_lock;

        // List of shared workers. Expectation is that there will be a limited number of shared workers, and so tracking them in a Vector is more efficient than nested HashMaps.
        typedef Vector<RefPtr<SharedWorkerProxy> > SharedWorkerProxyRepository;
        SharedWorkerProxyRepository m_proxies;
    };

} // namespace WebCore

#endif // ENABLE(SHARED_WORKERS)

#endif // DefaultSharedWorkerRepository_h
