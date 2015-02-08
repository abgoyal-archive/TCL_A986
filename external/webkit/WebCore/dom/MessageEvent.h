

#ifndef MessageEvent_h
#define MessageEvent_h

#include "DOMWindow.h"
#include "Event.h"
#include "MessagePort.h"
#include "SerializedScriptValue.h"

namespace WebCore {

    class DOMWindow;

    class MessageEvent : public Event {
    public:
        static PassRefPtr<MessageEvent> create()
        {
            return adoptRef(new MessageEvent);
        }
        static PassRefPtr<MessageEvent> create(PassOwnPtr<MessagePortArray> ports, PassRefPtr<SerializedScriptValue> data = 0, const String& origin = "", const String& lastEventId = "", PassRefPtr<DOMWindow> source = 0)
        {
            return adoptRef(new MessageEvent(data, origin, lastEventId, source, ports));
        }
        virtual ~MessageEvent();

        void initMessageEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<SerializedScriptValue> data, const String& origin, const String& lastEventId, DOMWindow* source, PassOwnPtr<MessagePortArray>);

        SerializedScriptValue* data() const { return m_data.get(); }
        const String& origin() const { return m_origin; }
        const String& lastEventId() const { return m_lastEventId; }
        DOMWindow* source() const { return m_source.get(); }
        MessagePortArray* ports() const { return m_ports.get(); }

        // FIXME: remove this when we update the ObjC bindings (bug #28774).
        MessagePort* messagePort();
        // FIXME: remove this when we update the ObjC bindings (bug #28774).
        void initMessageEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<SerializedScriptValue> data, const String& origin, const String& lastEventId, DOMWindow* source, MessagePort*);

        virtual bool isMessageEvent() const;

    private:
        MessageEvent();
        MessageEvent(PassRefPtr<SerializedScriptValue> data, const String& origin, const String& lastEventId, PassRefPtr<DOMWindow> source, PassOwnPtr<MessagePortArray>);

        RefPtr<SerializedScriptValue> m_data;
        String m_origin;
        String m_lastEventId;
        RefPtr<DOMWindow> m_source;
        OwnPtr<MessagePortArray> m_ports;
    };

} // namespace WebCore

#endif // MessageEvent_h
