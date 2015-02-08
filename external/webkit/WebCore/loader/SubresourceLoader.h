

#ifndef SubresourceLoader_h
#define SubresourceLoader_h

#include "FrameLoaderTypes.h"
#include "ResourceLoader.h"
 
namespace WebCore {

    class ResourceRequest;
    class SubresourceLoaderClient;
    
    class SubresourceLoader : public ResourceLoader {
    public:
        static PassRefPtr<SubresourceLoader> create(Frame*, SubresourceLoaderClient*, const ResourceRequest&, SecurityCheckPolicy = DoSecurityCheck, bool sendResourceLoadCallbacks = true, bool shouldContentSniff = true);

        void clearClient() { m_client = 0; }

    private:
        SubresourceLoader(Frame*, SubresourceLoaderClient*, bool sendResourceLoadCallbacks, bool shouldContentSniff);
        virtual ~SubresourceLoader();
        
        virtual void willSendRequest(ResourceRequest&, const ResourceResponse& redirectResponse);
        virtual void didSendData(unsigned long long bytesSent, unsigned long long totalBytesToBeSent);
        virtual void didReceiveResponse(const ResourceResponse&);
        virtual void didReceiveData(const char*, int, long long lengthReceived, bool allAtOnce);
        virtual void didFinishLoading();
        virtual void didFail(const ResourceError&);
        virtual bool shouldUseCredentialStorage();
        virtual void didReceiveAuthenticationChallenge(const AuthenticationChallenge&);
        virtual void receivedCancellation(const AuthenticationChallenge&);        
        virtual void didCancel(const ResourceError&);

        SubresourceLoaderClient* m_client;
        bool m_loadingMultipartContent;
    };

}

#endif // SubresourceLoader_h
