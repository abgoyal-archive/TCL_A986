

#ifndef WebSocketStreamHandleClient_h
#define WebSocketStreamHandleClient_h

#include "WebCommon.h"

namespace WebKit {

class WebData;
class WebSocketStreamError;
class WebSocketStreamHandle;
class WebURL;

class WebSocketStreamHandleClient {
public:

    // Called when Socket Stream is opened.
    virtual void didOpenStream(WebSocketStreamHandle*, int /* maxPendingSendAllowed */) = 0;

    // Called when |amountSent| bytes are sent.
    virtual void didSendData(WebSocketStreamHandle*, int /* amountSent */) = 0;

    // Called when data are received.
    virtual void didReceiveData(WebSocketStreamHandle*, const WebData&) = 0;

    // Called when Socket Stream is closed.
    virtual void didClose(WebSocketStreamHandle*) = 0;

    // Called when Socket Stream has an error.
    virtual void didFail(WebSocketStreamHandle*, const WebSocketStreamError&) = 0;

    // FIXME: auth challenge for proxy
};

} // namespace WebKit

#endif
