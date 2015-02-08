

#ifndef ApplicationCacheHost_h
#define ApplicationCacheHost_h

#if ENABLE(OFFLINE_WEB_APPLICATIONS)

#include <wtf/Deque.h>
#include <wtf/OwnPtr.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

    class DOMApplicationCache;
    class DocumentLoader;
    class KURL;
    class ResourceLoader;
    class ResourceError;
    class ResourceRequest;
    class ResourceResponse;
    class SubstituteData;
#if PLATFORM(CHROMIUM)
    class ApplicationCacheHostInternal;
#else
    class ApplicationCache;
    class ApplicationCacheGroup;
    class ApplicationCacheResource;
    class ApplicationCacheStorage;
#endif

    class ApplicationCacheHost : public Noncopyable {
    public:
        // The Status numeric values are specified in the HTML5 spec.
        enum Status {
            UNCACHED = 0,
            IDLE = 1,
            CHECKING = 2,
            DOWNLOADING = 3,
            UPDATEREADY = 4,
            OBSOLETE = 5
        };

        enum EventID {
            CHECKING_EVENT = 0,
            ERROR_EVENT,
            NOUPDATE_EVENT,
            DOWNLOADING_EVENT,
            PROGRESS_EVENT,
            UPDATEREADY_EVENT,
            CACHED_EVENT,
            OBSOLETE_EVENT  // Must remain the last value, this is used to size arrays.
        };

        ApplicationCacheHost(DocumentLoader*);
        ~ApplicationCacheHost();

        void selectCacheWithoutManifest();
        void selectCacheWithManifest(const KURL& manifestURL);

        void maybeLoadMainResource(ResourceRequest&, SubstituteData&);
        bool maybeLoadFallbackForMainResponse(const ResourceRequest&, const ResourceResponse&);
        bool maybeLoadFallbackForMainError(const ResourceRequest&, const ResourceError&);
        void mainResourceDataReceived(const char* data, int length, long long lengthReceived, bool allAtOnce);
        void finishedLoadingMainResource();
        void failedLoadingMainResource();

        bool maybeLoadResource(ResourceLoader*, ResourceRequest&, const KURL& originalURL);
        bool maybeLoadFallbackForRedirect(ResourceLoader*, ResourceRequest&, const ResourceResponse&);
        bool maybeLoadFallbackForResponse(ResourceLoader*, const ResourceResponse&);
        bool maybeLoadFallbackForError(ResourceLoader*, const ResourceError&);

        bool maybeLoadSynchronously(ResourceRequest&, ResourceError&, ResourceResponse&, Vector<char>& data);
        void maybeLoadFallbackSynchronously(const ResourceRequest&, ResourceError&, ResourceResponse&, Vector<char>& data);

        bool canCacheInPageCache() const;

        Status status() const;  
        bool update();
        bool swapCache();

        void setDOMApplicationCache(DOMApplicationCache* domApplicationCache);
        void notifyDOMApplicationCache(EventID id);

        void stopDeferringEvents(); // Also raises the events that have been queued up.

    private:
        bool isApplicationCacheEnabled();
        DocumentLoader* documentLoader() { return m_documentLoader; }

        DOMApplicationCache* m_domApplicationCache;
        DocumentLoader* m_documentLoader;
        bool m_defersEvents; // Events are deferred until after document onload.
        Vector<EventID> m_deferredEvents;

#if PLATFORM(CHROMIUM)
        friend class ApplicationCacheHostInternal;
        OwnPtr<ApplicationCacheHostInternal> m_internal;
#else
        friend class ApplicationCacheGroup;
        friend class ApplicationCacheStorage;

        bool scheduleLoadFallbackResourceFromApplicationCache(ResourceLoader*, ApplicationCache* = 0);
        bool shouldLoadResourceFromApplicationCache(const ResourceRequest&, ApplicationCacheResource*&);
        bool getApplicationCacheFallbackResource(const ResourceRequest&, ApplicationCacheResource*&, ApplicationCache* = 0);
        void setCandidateApplicationCacheGroup(ApplicationCacheGroup* group);
        ApplicationCacheGroup* candidateApplicationCacheGroup() const { return m_candidateApplicationCacheGroup; }
        void setApplicationCache(PassRefPtr<ApplicationCache> applicationCache);
        ApplicationCache* applicationCache() const { return m_applicationCache.get(); }
        ApplicationCache* mainResourceApplicationCache() const { return m_mainResourceApplicationCache.get(); }


        // The application cache that the document loader is associated with (if any).
        RefPtr<ApplicationCache> m_applicationCache;

        // Before an application cache has finished loading, this will be the candidate application
        // group that the document loader is associated with.
        ApplicationCacheGroup* m_candidateApplicationCacheGroup;

        // This is the application cache the main resource was loaded from (if any).
        RefPtr<ApplicationCache> m_mainResourceApplicationCache;
#endif
    };

}  // namespace WebCore

#endif  // ENABLE(OFFLINE_WEB_APPLICATIONS)
#endif  // ApplicationCacheHost_h
