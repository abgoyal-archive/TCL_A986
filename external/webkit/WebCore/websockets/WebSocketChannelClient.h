

#ifndef WebSocketChannelClient_h
#define WebSocketChannelClient_h

#if ENABLE(WEB_SOCKETS)

namespace WebCore {

    class WebSocketChannelClient {
    public:
        virtual ~WebSocketChannelClient() { }
        virtual void didConnect() { }
        virtual void didReceiveMessage(const String&) { }
        virtual void didClose(unsigned long /* unhandledBufferedAmount */) { }

    protected:
        WebSocketChannelClient() { }
    };

} // namespace WebCore

#endif // ENABLE(WEB_SOCKETS)

#endif // WebSocketChannelClient_h
