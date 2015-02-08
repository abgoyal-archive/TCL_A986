

#ifndef WebMessagePortChannel_h
#define WebMessagePortChannel_h

#include "WebCommon.h"
#include "WebVector.h"

namespace WebKit {

class WebMessagePortChannelClient;
class WebString;

typedef WebVector<class WebMessagePortChannel*> WebMessagePortChannelArray;

// Provides an interface to a Message Port Channel implementation.  The object owns itself and
// is signalled that its not needed anymore with the destroy() call.
class WebMessagePortChannel {
public:
    virtual void setClient(WebMessagePortChannelClient*) = 0;
    virtual void destroy() = 0;
    // WebKit versions of WebCore::MessagePortChannel.
    virtual void entangle(WebMessagePortChannel*) = 0;
    // Callee receives ownership of the passed vector.
    virtual void postMessage(const WebString&, WebMessagePortChannelArray*) = 0;
    virtual bool tryGetMessage(WebString*, WebMessagePortChannelArray&) = 0;

protected:
    ~WebMessagePortChannel() { }
};

} // namespace WebKit

#endif
