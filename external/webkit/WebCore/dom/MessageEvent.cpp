

#include "config.h"
#include "MessageEvent.h"

#include "DOMWindow.h"
#include "EventNames.h"

namespace WebCore {

MessageEvent::MessageEvent()
    : m_data(SerializedScriptValue::create())
{
}

MessageEvent::MessageEvent(PassRefPtr<SerializedScriptValue> data, const String& origin, const String& lastEventId, PassRefPtr<DOMWindow> source, PassOwnPtr<MessagePortArray> ports)
    : Event(eventNames().messageEvent, false, false)
    , m_data(data)
    , m_origin(origin)
    , m_lastEventId(lastEventId)
    , m_source(source)
    , m_ports(ports)
{
}

MessageEvent::~MessageEvent()
{
}

void MessageEvent::initMessageEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<SerializedScriptValue> data, const String& origin, const String& lastEventId, DOMWindow* source, PassOwnPtr<MessagePortArray> ports)
{
    if (dispatched())
        return;
        
    initEvent(type, canBubble, cancelable);
    
    m_data = data;
    m_origin = origin;
    m_lastEventId = lastEventId;
    m_source = source;
    m_ports = ports;
}

// FIXME: remove this when we update the ObjC bindings (bug #28774).
MessagePort* MessageEvent::messagePort()
{
    if (!m_ports)
        return 0;
    ASSERT(m_ports->size() == 1);
    return (*m_ports)[0].get();
}

void MessageEvent::initMessageEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<SerializedScriptValue> data, const String& origin, const String& lastEventId, DOMWindow* source, MessagePort* port)
{
    MessagePortArray* ports = 0;
    if (port) {
        ports = new MessagePortArray();
        ports->append(port);
    }
    initMessageEvent(type, canBubble, cancelable, data, origin, lastEventId, source, ports);
}

bool MessageEvent::isMessageEvent() const 
{
    return true;
}

} // namespace WebCore
