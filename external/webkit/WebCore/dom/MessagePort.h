

#ifndef MessagePort_h
#define MessagePort_h

#include "AtomicStringHash.h"
#include "EventListener.h"
#include "EventNames.h"
#include "EventTarget.h"
#include "MessagePortChannel.h"
#include <wtf/HashMap.h>
#include <wtf/OwnPtr.h>
#include <wtf/PassOwnPtr.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

    class AtomicStringImpl;
    class Event;
    class Frame;
    class MessagePort;
    class ScriptExecutionContext;
    class String;

    // The overwhelmingly common case is sending a single port, so handle that efficiently with an inline buffer of size 1.
    typedef Vector<RefPtr<MessagePort>, 1> MessagePortArray;

    class MessagePort : public RefCounted<MessagePort>, public EventTarget {
    public:
        static PassRefPtr<MessagePort> create(ScriptExecutionContext& scriptExecutionContext) { return adoptRef(new MessagePort(scriptExecutionContext)); }
        ~MessagePort();

        void postMessage(PassRefPtr<SerializedScriptValue> message, ExceptionCode&);
        void postMessage(PassRefPtr<SerializedScriptValue> message, const MessagePortArray*, ExceptionCode&);
        // FIXME: remove this when we update the ObjC bindings (bug #28774).
        void postMessage(PassRefPtr<SerializedScriptValue> message, MessagePort*, ExceptionCode&);

        void start();
        void close();

        void entangle(PassOwnPtr<MessagePortChannel>);
        PassOwnPtr<MessagePortChannel> disentangle(ExceptionCode&);

        // Disentangle an array of ports, returning the entangled channels.
        // Per section 8.3.3 of the HTML5 spec, generates an INVALID_STATE_ERR exception if any of the passed ports are null or not entangled.
        // Returns 0 if there is an exception, or if the passed-in array is 0/empty.
        static PassOwnPtr<MessagePortChannelArray> disentanglePorts(const MessagePortArray*, ExceptionCode&);

        // Entangles an array of channels, returning an array of MessagePorts in matching order.
        // Returns 0 if the passed array is 0/empty.
        static PassOwnPtr<MessagePortArray> entanglePorts(ScriptExecutionContext&, PassOwnPtr<MessagePortChannelArray>);

        void messageAvailable();
        bool started() const { return m_started; }

        void contextDestroyed();

        virtual ScriptExecutionContext* scriptExecutionContext() const;

        virtual MessagePort* toMessagePort() { return this; }

        void dispatchMessages();

        using RefCounted<MessagePort>::ref;
        using RefCounted<MessagePort>::deref;

        bool hasPendingActivity();

        void setOnmessage(PassRefPtr<EventListener> listener)
        {
            setAttributeEventListener(eventNames().messageEvent, listener);
            start();
        }
        EventListener* onmessage() { return getAttributeEventListener(eventNames().messageEvent); }

        // Returns null if there is no entangled port, or if the entangled port is run by a different thread.
        // Returns null otherwise.
        // NOTE: This is used solely to enable a GC optimization. Some platforms may not be able to determine ownership of the remote port (since it may live cross-process) - those platforms may always return null.
        MessagePort* locallyEntangledPort();
        // A port starts out its life entangled, and remains entangled until it is closed or is cloned.
        bool isEntangled() { return !m_closed && !isCloned(); }
        // A port is cloned if its entangled channel has been removed and sent to a new owner via postMessage().
        bool isCloned() { return !m_entangledChannel; }

    private:
        MessagePort(ScriptExecutionContext&);

        virtual void refEventTarget() { ref(); }
        virtual void derefEventTarget() { deref(); }
        virtual EventTargetData* eventTargetData();
        virtual EventTargetData* ensureEventTargetData();

        OwnPtr<MessagePortChannel> m_entangledChannel;

        bool m_started;
        bool m_closed;

        ScriptExecutionContext* m_scriptExecutionContext;
        EventTargetData m_eventTargetData;
    };

} // namespace WebCore

#endif // MessagePort_h
