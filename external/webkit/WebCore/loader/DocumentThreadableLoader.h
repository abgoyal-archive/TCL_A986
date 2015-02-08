

#ifndef DocumentThreadableLoader_h
#define DocumentThreadableLoader_h

#include "FrameLoaderTypes.h"
#include "SubresourceLoaderClient.h"
#include "ThreadableLoader.h"
#include <wtf/OwnPtr.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {
    class Document;
    class KURL;
    class ResourceRequest;
    class ThreadableLoaderClient;

    class DocumentThreadableLoader : public RefCounted<DocumentThreadableLoader>, public ThreadableLoader, private SubresourceLoaderClient  {
    public:
        static void loadResourceSynchronously(Document*, const ResourceRequest&, ThreadableLoaderClient&, const ThreadableLoaderOptions&);
        static PassRefPtr<DocumentThreadableLoader> create(Document*, ThreadableLoaderClient*, const ResourceRequest&, const ThreadableLoaderOptions&);
        virtual ~DocumentThreadableLoader();

        virtual void cancel();

        using RefCounted<DocumentThreadableLoader>::ref;
        using RefCounted<DocumentThreadableLoader>::deref;

    protected:
        virtual void refThreadableLoader() { ref(); }
        virtual void derefThreadableLoader() { deref(); }

    private:
        enum BlockingBehavior {
            LoadSynchronously,
            LoadAsynchronously
        };

        DocumentThreadableLoader(Document*, ThreadableLoaderClient*, BlockingBehavior blockingBehavior, const ResourceRequest&, const ThreadableLoaderOptions& options);

        virtual void willSendRequest(SubresourceLoader*, ResourceRequest&, const ResourceResponse& redirectResponse);
        virtual void didSendData(SubresourceLoader*, unsigned long long bytesSent, unsigned long long totalBytesToBeSent);

        virtual void didReceiveResponse(SubresourceLoader*, const ResourceResponse&);
        virtual void didReceiveData(SubresourceLoader*, const char*, int lengthReceived);
        virtual void didFinishLoading(SubresourceLoader*);
        virtual void didFail(SubresourceLoader*, const ResourceError&);

        virtual bool getShouldUseCredentialStorage(SubresourceLoader*, bool& shouldUseCredentialStorage);
        virtual void didReceiveAuthenticationChallenge(SubresourceLoader*, const AuthenticationChallenge&);
        virtual void receivedCancellation(SubresourceLoader*, const AuthenticationChallenge&);

        void didFinishLoading(unsigned long identifier);
        void makeSimpleCrossOriginAccessRequest(const ResourceRequest& request);
        void makeCrossOriginAccessRequestWithPreflight(const ResourceRequest& request);
        void preflightSuccess();
        void preflightFailure();

        void loadRequest(const ResourceRequest&, SecurityCheckPolicy);
        bool isAllowedRedirect(const KURL&);

        RefPtr<SubresourceLoader> m_loader;
        ThreadableLoaderClient* m_client;
        Document* m_document;
        ThreadableLoaderOptions m_options;
        bool m_sameOriginRequest;
        bool m_async;
        OwnPtr<ResourceRequest> m_actualRequest;  // non-null during Access Control preflight checks
    };

} // namespace WebCore

#endif // DocumentThreadableLoader_h
