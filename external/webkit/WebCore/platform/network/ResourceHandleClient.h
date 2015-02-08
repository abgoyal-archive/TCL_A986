

#ifndef ResourceHandleClient_h
#define ResourceHandleClient_h

#include <wtf/RefCounted.h>
#include <wtf/Platform.h>
#include <wtf/RefPtr.h>

#if USE(CFNETWORK)
#include <ConditionalMacros.h>
#include <CFNetwork/CFURLCachePriv.h>
#include <CFNetwork/CFURLResponsePriv.h>
#endif

#if PLATFORM(MAC)
#ifdef __OBJC__
@class NSCachedURLResponse;
#else
class NSCachedURLResponse;
#endif
#endif

namespace WebCore {
    class AuthenticationChallenge;
    class Credential;
    class KURL;
    class ResourceHandle;
    class ResourceError;
    class ResourceRequest;
    class ResourceResponse;

    enum CacheStoragePolicy {
        StorageAllowed,
        StorageAllowedInMemoryOnly,
        StorageNotAllowed,
    };
    
    class ResourceHandleClient {
    public:
        virtual ~ResourceHandleClient() { }

        // request may be modified
        virtual void willSendRequest(ResourceHandle*, ResourceRequest&, const ResourceResponse& /*redirectResponse*/) { }
        virtual void didSendData(ResourceHandle*, unsigned long long /*bytesSent*/, unsigned long long /*totalBytesToBeSent*/) { }

        virtual void didReceiveResponse(ResourceHandle*, const ResourceResponse&) { }
        virtual void didReceiveData(ResourceHandle*, const char*, int, int /*lengthReceived*/) { }
        virtual void didFinishLoading(ResourceHandle*) { }
        virtual void didFail(ResourceHandle*, const ResourceError&) { }
        virtual void wasBlocked(ResourceHandle*) { }
        virtual void cannotShowURL(ResourceHandle*) { }

        virtual void willCacheResponse(ResourceHandle*, CacheStoragePolicy&) { }

        virtual bool shouldUseCredentialStorage(ResourceHandle*) { return false; }
        virtual void didReceiveAuthenticationChallenge(ResourceHandle*, const AuthenticationChallenge&) { }
        virtual void didCancelAuthenticationChallenge(ResourceHandle*, const AuthenticationChallenge&) { }
        virtual void receivedCancellation(ResourceHandle*, const AuthenticationChallenge&) { }

#if PLATFORM(MAC)        
        virtual NSCachedURLResponse* willCacheResponse(ResourceHandle*, NSCachedURLResponse* response) { return response; }
        virtual void willStopBufferingData(ResourceHandle*, const char*, int) { } 
#endif
#if USE(CFNETWORK)
        virtual bool shouldCacheResponse(ResourceHandle*, CFCachedURLResponseRef response) { return true; }
#endif
    };

}

#endif
