

#ifndef EventSource_h
#define EventSource_h

#if ENABLE(EVENTSOURCE)

#include "ActiveDOMObject.h"
#include "AtomicStringHash.h"
#include "EventNames.h"
#include "EventTarget.h"
#include "KURL.h"
#include "ThreadableLoaderClient.h"
#include "Timer.h"

#include <wtf/HashMap.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

    class MessageEvent;
    class ResourceResponse;
    class TextResourceDecoder;
    class ThreadableLoader;

    class EventSource : public RefCounted<EventSource>, public EventTarget, private ThreadableLoaderClient, public ActiveDOMObject {
    public:
        static PassRefPtr<EventSource> create(const String& url, ScriptExecutionContext* context, ExceptionCode& ec) { return adoptRef(new EventSource(url, context, ec)); }
        virtual ~EventSource();

        static const unsigned long long defaultReconnectDelay;

        String url() const;

        enum State {
            CONNECTING = 0,
            OPEN = 1,
            CLOSED = 2,
        };

        State readyState() const;

        DEFINE_ATTRIBUTE_EVENT_LISTENER(open);
        DEFINE_ATTRIBUTE_EVENT_LISTENER(message);
        DEFINE_ATTRIBUTE_EVENT_LISTENER(error);

        void close();

        using RefCounted<EventSource>::ref;
        using RefCounted<EventSource>::deref;

        virtual EventSource* toEventSource() { return this; }
        virtual ScriptExecutionContext* scriptExecutionContext() const;

        virtual void stop();

    private:
        EventSource(const String& url, ScriptExecutionContext* context, ExceptionCode& ec);

        virtual void refEventTarget() { ref(); }
        virtual void derefEventTarget() { deref(); }
        virtual EventTargetData* eventTargetData();
        virtual EventTargetData* ensureEventTargetData();

        virtual void didReceiveResponse(const ResourceResponse& response);
        virtual void didReceiveData(const char* data, int length);
        virtual void didFinishLoading(unsigned long);
        virtual void didFail(const ResourceError& error);
        virtual void didFailRedirectCheck();

        void connect();
        void endRequest();
        void scheduleReconnect();
        void reconnectTimerFired(Timer<EventSource>*);
        void parseEventStream();
        void parseEventStreamLine(unsigned int pos, int fieldLength, int lineLength);
        PassRefPtr<MessageEvent> createMessageEvent();

        KURL m_url;
        State m_state;

        RefPtr<TextResourceDecoder> m_decoder;
        RefPtr<ThreadableLoader> m_loader;
        Timer<EventSource> m_reconnectTimer;
        Vector<UChar> m_receiveBuf;
        bool m_discardTrailingNewline;
        bool m_failSilently;
        bool m_requestInFlight;

        String m_eventName;
        Vector<UChar> m_data;
        String m_lastEventId;
        unsigned long long m_reconnectDelay;
        String m_origin;
        
        EventTargetData m_eventTargetData;
    };

} // namespace WebCore

#endif // ENABLE(EVENTSOURCE)

#endif // EventSource_h
