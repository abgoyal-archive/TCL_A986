

#ifndef WebSocketChannel_h
#define WebSocketChannel_h

#if ENABLE(WEB_SOCKETS)

#include "SocketStreamHandleClient.h"
#include "ThreadableWebSocketChannel.h"
#include "WebSocketHandshake.h"
#include <wtf/RefCounted.h>

namespace WebCore {

    class ScriptExecutionContext;
    class String;
    class SocketStreamHandle;
    class SocketStreamError;
    class WebSocketChannelClient;

    class WebSocketChannel : public RefCounted<WebSocketChannel>, public SocketStreamHandleClient, public ThreadableWebSocketChannel {
    public:
        static PassRefPtr<WebSocketChannel> create(ScriptExecutionContext* context, WebSocketChannelClient* client, const KURL& url, const String& protocol) { return adoptRef(new WebSocketChannel(context, client, url, protocol)); }
        virtual ~WebSocketChannel();

        virtual void connect();
        virtual bool send(const String& message);
        virtual unsigned long bufferedAmount() const;
        virtual void close();
        virtual void disconnect(); // Will suppress didClose().

        virtual void didOpen(SocketStreamHandle*);
        virtual void didClose(SocketStreamHandle*);
        virtual void didReceiveData(SocketStreamHandle*, const char*, int);
        virtual void didFail(SocketStreamHandle*, const SocketStreamError&);
        virtual void didReceiveAuthenticationChallenge(SocketStreamHandle*, const AuthenticationChallenge&);
        virtual void didCancelAuthenticationChallenge(SocketStreamHandle*, const AuthenticationChallenge&);

        using RefCounted<WebSocketChannel>::ref;
        using RefCounted<WebSocketChannel>::deref;

    protected:
        virtual void refThreadableWebSocketChannel() { ref(); }
        virtual void derefThreadableWebSocketChannel() { deref(); }

    private:
        WebSocketChannel(ScriptExecutionContext*, WebSocketChannelClient*, const KURL&, const String& protocol);

        bool appendToBuffer(const char* data, int len);
        void skipBuffer(int len);

        ScriptExecutionContext* m_context;
        WebSocketChannelClient* m_client;
        WebSocketHandshake m_handshake;
        RefPtr<SocketStreamHandle> m_handle;
        char* m_buffer;
        int m_bufferSize;
    };

} // namespace WebCore

#endif // ENABLE(WEB_SOCKETS)

#endif // WebSocketChannel_h
