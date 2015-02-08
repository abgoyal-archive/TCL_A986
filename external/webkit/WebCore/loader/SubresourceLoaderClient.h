

#ifndef SubresourceLoaderClient_h
#define SubresourceLoaderClient_h

namespace WebCore {

class AuthenticationChallenge;
class ResourceError;
class ResourceRequest;
class ResourceResponse;
class SubresourceLoader;
    
class SubresourceLoaderClient {
public:
    virtual ~SubresourceLoaderClient() { } 

    // request may be modified
    virtual void willSendRequest(SubresourceLoader*, ResourceRequest&, const ResourceResponse& /*redirectResponse*/) { }
    virtual void didSendData(SubresourceLoader*, unsigned long long /*bytesSent*/, unsigned long long /*totalBytesToBeSent*/) { }

    virtual void didReceiveResponse(SubresourceLoader*, const ResourceResponse&) { }
    virtual void didReceiveData(SubresourceLoader*, const char*, int /*lengthReceived*/) { }
    virtual void didFinishLoading(SubresourceLoader*) { }
    virtual void didFail(SubresourceLoader*, const ResourceError&) { }
    
    virtual bool getShouldUseCredentialStorage(SubresourceLoader*, bool& /*shouldUseCredentialStorage*/) { return false; }
    virtual void didReceiveAuthenticationChallenge(SubresourceLoader*, const AuthenticationChallenge&) { }
    virtual void receivedCancellation(SubresourceLoader*, const AuthenticationChallenge&) { }

};

} // namespace WebCore

#endif // SubresourceLoaderClient_h
