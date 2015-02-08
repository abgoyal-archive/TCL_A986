

#ifndef SocketStreamHandle_h
#define SocketStreamHandle_h

#include "AuthenticationClient.h"
#include "SocketStreamHandleBase.h"
#include <wtf/RetainPtr.h>

namespace WebCore {

    class AuthenticationChallenge;
    class Credential;
    class SocketStreamHandleClient;

    class SocketStreamHandle : public RefCounted<SocketStreamHandle>, public SocketStreamHandleBase, public AuthenticationClient {
    public:
        static PassRefPtr<SocketStreamHandle> create(const KURL& url, SocketStreamHandleClient* client) { return adoptRef(new SocketStreamHandle(url, client)); }

        virtual ~SocketStreamHandle();

        using RefCounted<SocketStreamHandle>::ref;
        using RefCounted<SocketStreamHandle>::deref;

    private:
        virtual int platformSend(const char* data, int length);
        virtual void platformClose();

        SocketStreamHandle(const KURL&, SocketStreamHandleClient*);
        void createStreams();
        void scheduleStreams();
        void chooseProxy();
#ifndef BUILDING_ON_TIGER
        void chooseProxyFromArray(CFArrayRef);
        void executePACFileURL(CFURLRef);
        void removePACRunLoopSource();
        RetainPtr<CFRunLoopSourceRef> m_pacRunLoopSource;
        static void pacExecutionCallback(void* client, CFArrayRef proxyList, CFErrorRef error);
        static void pacExecutionCallbackMainThread(void*);
        static CFStringRef copyPACExecutionDescription(void*);
#endif

        bool shouldUseSSL() const { return m_url.protocolIs("wss"); }

        void addCONNECTCredentials(CFHTTPMessageRef response);

        static CFStringRef copyCFStreamDescription(void* );
        static void readStreamCallback(CFReadStreamRef, CFStreamEventType, void*);
        static void writeStreamCallback(CFWriteStreamRef, CFStreamEventType, void*);
#if PLATFORM(WIN)
        static void readStreamCallbackMainThread(void*);
        static void writeStreamCallbackMainThread(void*);
#endif
        void readStreamCallback(CFStreamEventType);
        void writeStreamCallback(CFStreamEventType);

        // No authentication for streams per se, but proxy may ask for credentials.
        virtual void receivedCredential(const AuthenticationChallenge&, const Credential&);
        virtual void receivedRequestToContinueWithoutCredential(const AuthenticationChallenge&);
        virtual void receivedCancellation(const AuthenticationChallenge&);

        virtual void refAuthenticationClient() { ref(); }
        virtual void derefAuthenticationClient() { deref(); }

        enum ConnectingSubstate { New, ExecutingPACFile, WaitingForCredentials, WaitingForConnect, Connected };
        ConnectingSubstate m_connectingSubstate;

        enum ConnectionType { Unknown, Direct, SOCKSProxy, CONNECTProxy };
        ConnectionType m_connectionType;
        RetainPtr<CFStringRef> m_proxyHost;
        RetainPtr<CFNumberRef> m_proxyPort;

        RetainPtr<CFHTTPMessageRef> m_proxyResponseMessage;
        bool m_sentStoredCredentials;
        RetainPtr<CFReadStreamRef> m_readStream;
        RetainPtr<CFWriteStreamRef> m_writeStream;

        RetainPtr<CFURLRef> m_httpsURL; // ws(s): replaced with https:
    };

}  // namespace WebCore

#endif  // SocketStreamHandle_h
