

#ifndef WebSocketStreamHandle_h
#define WebSocketStreamHandle_h

#include "WebCommon.h"

namespace WebKit {

class WebData;
class WebSocketStreamHandleClient;
class WebURL;

class WebSocketStreamHandle {
public:
    virtual ~WebSocketStreamHandle() { }

    // Connect new socket stream asynchronously.
    virtual void connect(const WebURL&, WebSocketStreamHandleClient*) = 0;

    // Send web socket frame data on the socket stream.
    virtual bool send(const WebData&) = 0;

    // Close the socket stream.
    virtual void close() = 0;
};

} // namespace WebKit

#endif
