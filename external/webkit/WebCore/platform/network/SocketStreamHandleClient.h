

#ifndef SocketStreamHandleClient_h
#define SocketStreamHandleClient_h

namespace WebCore {

    class AuthenticationChallenge;
    class KURL;
    class SocketStreamError;
    class SocketStreamHandle;

    class SocketStreamHandleClient {
    public:
        virtual ~SocketStreamHandleClient() { }

        virtual void didOpen(SocketStreamHandle*) { }
        virtual void didClose(SocketStreamHandle*) { }
        virtual void didReceiveData(SocketStreamHandle*, const char* /*data*/, int /*length*/) { }

        virtual void didFail(SocketStreamHandle*, const SocketStreamError&) { }

        // No authentication for streams per se, but proxy may ask for credentials.
        virtual void didReceiveAuthenticationChallenge(SocketStreamHandle*, const AuthenticationChallenge&) { }
        virtual void didCancelAuthenticationChallenge(SocketStreamHandle*, const AuthenticationChallenge&) { }
    };

}  // namespace WebCore

#endif  // SocketStreamHandleClient_h
