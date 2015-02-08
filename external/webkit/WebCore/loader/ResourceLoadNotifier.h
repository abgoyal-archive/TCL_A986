

#ifndef ResourceLoadNotifier_h
#define ResourceLoadNotifier_h

#include <wtf/Noncopyable.h>

namespace WebCore {

class AuthenticationChallenge;
class DocumentLoader;
class Frame;
class ResourceError;
class ResourceLoader;
class ResourceResponse;
class ScriptString;
class ResourceRequest;

class ResourceLoadNotifier : public Noncopyable {
public:
    ResourceLoadNotifier(Frame*);

    void didReceiveAuthenticationChallenge(ResourceLoader*, const AuthenticationChallenge&);
    void didCancelAuthenticationChallenge(ResourceLoader*, const AuthenticationChallenge&);

    void willSendRequest(ResourceLoader*, ResourceRequest&, const ResourceResponse& redirectResponse);
    void didReceiveResponse(ResourceLoader*, const ResourceResponse&);
    void didReceiveData(ResourceLoader*, const char*, int, int lengthReceived);
    void didFinishLoad(ResourceLoader*);
    void didFailToLoad(ResourceLoader*, const ResourceError&);
    void didLoadResourceByXMLHttpRequest(unsigned long identifier, const ScriptString& sourceString);

    void assignIdentifierToInitialRequest(unsigned long identifier, DocumentLoader*, const ResourceRequest&);
    void dispatchWillSendRequest(DocumentLoader*, unsigned long identifier, ResourceRequest&, const ResourceResponse& redirectResponse);
    void dispatchDidReceiveResponse(DocumentLoader*, unsigned long identifier, const ResourceResponse&);
    void dispatchDidReceiveContentLength(DocumentLoader*, unsigned long identifier, int length);
    void dispatchDidFinishLoading(DocumentLoader*, unsigned long identifier);

    void sendRemainingDelegateMessages(DocumentLoader*, unsigned long identifier, const ResourceResponse&, int length, const ResourceError&);

private:
    Frame* m_frame;
};

} // namespace WebCore

#endif // ResourceLoadNotifier_h
