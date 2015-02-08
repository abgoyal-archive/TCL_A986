

#ifndef WebSocket_h
#define WebSocket_h

#if ENABLE(WEB_SOCKETS)

#include "ActiveDOMObject.h"
#include "AtomicStringHash.h"
#include "EventListener.h"
#include "EventNames.h"
#include "EventTarget.h"
#include "KURL.h"
#include "WebSocketChannelClient.h"
#include <wtf/OwnPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    class String;
    class ThreadableWebSocketChannel;

    class WebSocket : public RefCounted<WebSocket>, public EventTarget, public ActiveDOMObject, public WebSocketChannelClient {
    public:
#if USE(V8)
        static void setIsAvailable(bool);
        static bool isAvailable();
#endif
        static PassRefPtr<WebSocket> create(ScriptExecutionContext* context) { return adoptRef(new WebSocket(context)); }
        virtual ~WebSocket();

        enum State {
            CONNECTING = 0,
            OPEN = 1,
            CLOSED = 2
        };

        void connect(const KURL&, ExceptionCode&);
        void connect(const KURL&, const String& protocol, ExceptionCode&);

        bool send(const String& message, ExceptionCode&);

        void close();

        const KURL& url() const;
        State readyState() const;
        unsigned long bufferedAmount() const;

        DEFINE_ATTRIBUTE_EVENT_LISTENER(open);
        DEFINE_ATTRIBUTE_EVENT_LISTENER(message);
        DEFINE_ATTRIBUTE_EVENT_LISTENER(close);

        // EventTarget
        virtual WebSocket* toWebSocket() { return this; }

        virtual ScriptExecutionContext* scriptExecutionContext() const;
        virtual void contextDestroyed();
        virtual void stop();

        using RefCounted<WebSocket>::ref;
        using RefCounted<WebSocket>::deref;

        // WebSocketChannelClient
        virtual void didConnect();
        virtual void didReceiveMessage(const String& message);
        virtual void didClose(unsigned long unhandledBufferedAmount);

    private:
        WebSocket(ScriptExecutionContext*);

        virtual void refEventTarget() { ref(); }
        virtual void derefEventTarget() { deref(); }
        virtual EventTargetData* eventTargetData();
        virtual EventTargetData* ensureEventTargetData();

        void dispatchOpenEvent(Event*);
        void dispatchMessageEvent(Event*);
        void dispatchCloseEvent(Event*);

        RefPtr<ThreadableWebSocketChannel> m_channel;

        State m_state;
        KURL m_url;
        String m_protocol;
        EventTargetData m_eventTargetData;
        unsigned long m_bufferedAmountAfterClose;
    };

} // namespace WebCore

#endif // ENABLE(WEB_SOCKETS)

#endif // WebSocket_h
