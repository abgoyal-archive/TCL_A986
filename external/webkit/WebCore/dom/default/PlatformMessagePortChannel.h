

#ifndef PlatformMessagePortChannel_h
#define PlatformMessagePortChannel_h

#include "MessagePortChannel.h"

#include <wtf/MessageQueue.h>
#include <wtf/PassRefPtr.h>
#include <wtf/Threading.h>

namespace WebCore {

    class MessagePort;

    // PlatformMessagePortChannel is a platform-dependent interface to the remote side of a message channel.
    // This default implementation supports multiple threads running within a single process. Implementations for multi-process platforms should define these public APIs in their own platform-specific PlatformMessagePortChannel file.
    // The goal of this implementation is to eliminate contention except when cloning or closing the port, so each side of the channel has its own separate mutex.
    class PlatformMessagePortChannel : public ThreadSafeShared<PlatformMessagePortChannel> {
    public:
        static void createChannel(PassRefPtr<MessagePort>, PassRefPtr<MessagePort>);

        // APIs delegated from MessagePortChannel.h
        bool entangleIfOpen(MessagePort*);
        void disentangle();
        void postMessageToRemote(PassOwnPtr<MessagePortChannel::EventData>);
        bool tryGetMessageFromRemote(OwnPtr<MessagePortChannel::EventData>&);
        void close();
        bool isConnectedTo(MessagePort*);
        bool hasPendingActivity();
        MessagePort* locallyEntangledPort(const ScriptExecutionContext*);

        // Wrapper for MessageQueue that allows us to do thread safe sharing by two proxies.
        class MessagePortQueue : public ThreadSafeShared<MessagePortQueue> {
        public:
            static PassRefPtr<MessagePortQueue> create() { return adoptRef(new MessagePortQueue()); }

            PassOwnPtr<MessagePortChannel::EventData> tryGetMessage()
            {
                return m_queue.tryGetMessage();
            }

            bool appendAndCheckEmpty(PassOwnPtr<MessagePortChannel::EventData> message)
            {
                return m_queue.appendAndCheckEmpty(message);
            }

            bool isEmpty()
            {
                return m_queue.isEmpty();
            }

        private:
            MessagePortQueue() { }

            MessageQueue<MessagePortChannel::EventData> m_queue;
        };

        ~PlatformMessagePortChannel();

    private:
        static PassRefPtr<PlatformMessagePortChannel> create(PassRefPtr<MessagePortQueue> incoming, PassRefPtr<MessagePortQueue> outgoing);
        PlatformMessagePortChannel(PassRefPtr<MessagePortQueue> incoming, PassRefPtr<MessagePortQueue> outgoing);

        PassRefPtr<PlatformMessagePortChannel> entangledChannel();
        void setEntangledChannel(PassRefPtr<PlatformMessagePortChannel>);

        void setRemotePort(MessagePort*);
        MessagePort* remotePort();
        void closeInternal();

        // Mutex used to ensure exclusive access to the object internals.
        Mutex m_mutex;

        // Pointer to our entangled pair - cleared when close() is called.
        RefPtr<PlatformMessagePortChannel> m_entangledChannel;

        // Reference to the message queue for the (local) entangled port.
        RefPtr<MessagePortQueue> m_incomingQueue;
        RefPtr<MessagePortQueue> m_outgoingQueue;

        // The port we are connected to (the remote port) - this is the port that is notified when new messages arrive.
        MessagePort* m_remotePort;
    };

} // namespace WebCore

#endif // PlatformMessagePortChannel_h
