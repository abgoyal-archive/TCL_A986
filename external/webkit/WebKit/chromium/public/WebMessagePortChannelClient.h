

#ifndef WebMessagePortChannelClient_h
#define WebMessagePortChannelClient_h

namespace WebKit {

// Provides an interface for users of WebMessagePortChannel to be notified
// when messages are available.
class WebMessagePortChannelClient {
public:
    // Alerts that new messages have arrived, which are retrieved by calling
    // WebMessagePortChannel::tryGetMessage.  Note that this may be called
    // on any thread.
    virtual void messageAvailable() = 0;

protected:
    ~WebMessagePortChannelClient() { }
};

} // namespace WebKit

#endif
