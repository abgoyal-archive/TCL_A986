

#ifndef PlatformMessagePortChannel_h
#define PlatformMessagePortChannel_h

// FIXME: This relative path is a temporary hack to support using this
// header from webkit/glue.
#include "../public/WebMessagePortChannelClient.h"

#include "MessagePortChannel.h"

#include <wtf/PassRefPtr.h>
#include <wtf/Threading.h>

namespace WebKit {
class WebMessagePortChannel;
}

namespace WebCore {

class MessagePort;

// PlatformMessagePortChannel is a platform-dependent interface to the remote side of a message channel.
class PlatformMessagePortChannel : public ThreadSafeShared<PlatformMessagePortChannel>,
                                   public WebKit::WebMessagePortChannelClient {
public:
    static void createChannel(PassRefPtr<MessagePort>, PassRefPtr<MessagePort>);
    static PassRefPtr<PlatformMessagePortChannel> create();
    static PassRefPtr<PlatformMessagePortChannel> create(WebKit::WebMessagePortChannel*);

    // APIs delegated from MessagePortChannel.h
    bool entangleIfOpen(MessagePort*);
    void disentangle();
    void postMessageToRemote(PassOwnPtr<MessagePortChannel::EventData>);
    bool tryGetMessageFromRemote(OwnPtr<MessagePortChannel::EventData>&);
    void close();
    bool isConnectedTo(MessagePort* port);
    bool hasPendingActivity();

    // Releases ownership of the contained web channel.
    WebKit::WebMessagePortChannel* webChannelRelease();

    ~PlatformMessagePortChannel();

private:
    PlatformMessagePortChannel();
    PlatformMessagePortChannel(WebKit::WebMessagePortChannel*);

    void setEntangledChannel(PassRefPtr<PlatformMessagePortChannel>);

    // WebKit::WebMessagePortChannelClient implementation
    virtual void messageAvailable();

    // Mutex used to ensure exclusive access to the object internals.
    Mutex m_mutex;

    // Pointer to our entangled pair - cleared when close() is called.
    RefPtr<PlatformMessagePortChannel> m_entangledChannel;

    // The port we are connected to - this is the port that is notified when new messages arrive.
    MessagePort* m_localPort;

    WebKit::WebMessagePortChannel* m_webChannel;
};

} // namespace WebCore

#endif // PlatformMessagePortChannel_h
