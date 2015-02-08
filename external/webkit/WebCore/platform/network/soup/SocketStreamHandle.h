

#ifndef SocketStreamHandle_h
#define SocketStreamHandle_h

#include "SocketStreamHandleBase.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    class AuthenticationChallenge;
    class Credential;
    class SocketStreamHandleClient;

    class SocketStreamHandle : public RefCounted<SocketStreamHandle>, public SocketStreamHandleBase {
    public:
        static PassRefPtr<SocketStreamHandle> create(const KURL& url, SocketStreamHandleClient* client) { return adoptRef(new SocketStreamHandle(url, client)); }

        virtual ~SocketStreamHandle();

    protected:
        virtual int platformSend(const char* data, int length);
        virtual void platformClose();

    private:
        SocketStreamHandle(const KURL&, SocketStreamHandleClient*);

        // No authentication for streams per se, but proxy may ask for credentials.
        void didReceiveAuthenticationChallenge(const AuthenticationChallenge&);
        void receivedCredential(const AuthenticationChallenge&, const Credential&);
        void receivedRequestToContinueWithoutCredential(const AuthenticationChallenge&);
        void receivedCancellation(const AuthenticationChallenge&);
    };

}  // namespace WebCore

#endif  // SocketStreamHandle_h
