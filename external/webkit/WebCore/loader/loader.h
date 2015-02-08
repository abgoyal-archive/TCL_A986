

#ifndef loader_h
#define loader_h

#include "AtomicString.h"
#include "AtomicStringImpl.h"
#include "FrameLoaderTypes.h"
#include "PlatformString.h"
#include "SubresourceLoaderClient.h"
#include "Timer.h"
#include <wtf/Deque.h>
#include <wtf/HashMap.h>
#include <wtf/Noncopyable.h>

namespace WebCore {

    class CachedResource;
    class DocLoader;
    class KURL;
    class Request;

    class Loader : public Noncopyable {
    public:
        Loader();
        ~Loader();

        void load(DocLoader*, CachedResource*, bool incremental = true, SecurityCheckPolicy = DoSecurityCheck, bool sendResourceLoadCallbacks = true);

        void cancelRequests(DocLoader*);
        
        enum Priority { Low, Medium, High };
        void servePendingRequests(Priority minimumPriority = Low);

        bool isSuspendingPendingRequests() { return m_isSuspendingPendingRequests; }
        void suspendPendingRequests();
        void resumePendingRequests();
        
        void nonCacheRequestInFlight(const KURL&);
        void nonCacheRequestComplete(const KURL&);

    private:
        Priority determinePriority(const CachedResource*) const;
        void scheduleServePendingRequests();
        
        void requestTimerFired(Timer<Loader>*);

        class Host : public RefCounted<Host>, private SubresourceLoaderClient {
        public:
            static PassRefPtr<Host> create(const AtomicString& name, unsigned maxRequestsInFlight) 
            {
                return adoptRef(new Host(name, maxRequestsInFlight));
            }
            ~Host();
            
            const AtomicString& name() const { return m_name; }
            void addRequest(Request*, Priority);
            void nonCacheRequestInFlight();
            void nonCacheRequestComplete();
            void servePendingRequests(Priority minimumPriority = Low);
            void cancelRequests(DocLoader*);
            bool hasRequests() const;

            bool processingResource() const { return m_numResourcesProcessing != 0 || m_nonCachedRequestsInFlight !=0; }

        private:
            Host(const AtomicString&, unsigned);

            virtual void didReceiveResponse(SubresourceLoader*, const ResourceResponse&);
            virtual void didReceiveData(SubresourceLoader*, const char*, int);
            virtual void didFinishLoading(SubresourceLoader*);
            virtual void didFail(SubresourceLoader*, const ResourceError&);
            
            typedef Deque<Request*> RequestQueue;
            void servePendingRequests(RequestQueue& requestsPending, bool& serveLowerPriority);
            void didFail(SubresourceLoader*, bool cancelled = false);
            void cancelPendingRequests(RequestQueue& requestsPending, DocLoader*);
            
            RequestQueue m_requestsPending[High + 1];
            typedef HashMap<RefPtr<SubresourceLoader>, Request*> RequestMap;
            RequestMap m_requestsLoading;
            const AtomicString m_name;
            const int m_maxRequestsInFlight;
            int m_numResourcesProcessing;
            int m_nonCachedRequestsInFlight;
        };
        typedef HashMap<AtomicStringImpl*, RefPtr<Host> > HostMap;
        HostMap m_hosts;
        RefPtr<Host> m_nonHTTPProtocolHost;
        
        Timer<Loader> m_requestTimer;

        bool m_isSuspendingPendingRequests;
    };

}

#endif
