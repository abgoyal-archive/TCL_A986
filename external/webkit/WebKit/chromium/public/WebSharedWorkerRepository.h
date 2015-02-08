

#ifndef WebSharedWorkerRepository_h
#define WebSharedWorkerRepository_h

#include "WebCommon.h"

namespace WebKit {

class WebString;
class WebSharedWorker;
class WebURL;

class WebSharedWorkerRepository {
public:
    // Unique identifier for the parent document of a worker (unique within a given process).
    typedef unsigned long long DocumentID;

    // Tracks a newly-created SharedWorker via the repository.
    virtual void addSharedWorker(WebSharedWorker*, DocumentID) = 0;

    // Invoked when a document has been detached. DocumentID can be re-used after documentDetached() is invoked.
    virtual void documentDetached(DocumentID) = 0;

    // Returns true if the passed document is associated with any SharedWorkers.
    virtual bool hasSharedWorkers(DocumentID) = 0;
};

} // namespace WebKit

#endif // WebSharedWorkerRepository_h
