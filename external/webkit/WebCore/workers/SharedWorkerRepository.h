

#ifndef SharedWorkerRepository_h
#define SharedWorkerRepository_h

#if ENABLE(SHARED_WORKERS)

#include "ExceptionCode.h"
#include <wtf/PassOwnPtr.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

    class Document;
    class KURL;
    class MessagePortChannel;
    class SharedWorker;
    class String;

    // Interface to a repository which manages references to the set of active shared workers.
    class SharedWorkerRepository {
    public:
        // Returns true if the platform supports SharedWorkers, otherwise false.
        static bool isAvailable();

        // Connects the passed SharedWorker object with the specified worker thread, creating a new thread if necessary.
        static void connect(PassRefPtr<SharedWorker>, PassOwnPtr<MessagePortChannel>, const KURL&, const String& name, ExceptionCode&);

        // Invoked when a document has been detached.
        static void documentDetached(Document*);

        // Returns true if the passed document is associated with any SharedWorkers.
        static bool hasSharedWorkers(Document*);
    private:
        SharedWorkerRepository() { }
    };

} // namespace WebCore

#endif // ENABLE(SHARED_WORKERS)

#endif // SharedWorkerRepository_h
