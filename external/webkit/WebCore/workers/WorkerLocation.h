

#ifndef WorkerLocation_h
#define WorkerLocation_h

#if ENABLE(WORKERS)

#include "KURL.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {

    class String;

    class WorkerLocation : public RefCounted<WorkerLocation> {
    public:
        static PassRefPtr<WorkerLocation> create(const KURL& url)
        {
            return adoptRef(new WorkerLocation(url));
        }

        const KURL& url() const { return m_url; }

        String href() const;

        // URI decomposition attributes
        String protocol() const;
        String host() const;
        String hostname() const;
        String port() const;
        String pathname() const;
        String search() const;
        String hash() const;

        String toString() const;

    private:
        WorkerLocation(const KURL& url) : m_url(url) { }

        KURL m_url;
    };

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // WorkerLocation_h
