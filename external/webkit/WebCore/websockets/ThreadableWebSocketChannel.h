

#ifndef ThreadableWebSocketChannel_h
#define ThreadableWebSocketChannel_h

#if ENABLE(WEB_SOCKETS)

#include <wtf/Noncopyable.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

class KURL;
class ScriptExecutionContext;
class String;
class WebSocketChannelClient;

class ThreadableWebSocketChannel : public Noncopyable {
public:
    static PassRefPtr<ThreadableWebSocketChannel> create(ScriptExecutionContext*, WebSocketChannelClient*, const KURL&, const String& protocol);

    virtual void connect() = 0;
    virtual bool send(const String& message) = 0;
    virtual unsigned long bufferedAmount() const = 0;
    virtual void close() = 0;
    virtual void disconnect() = 0; // Will suppress didClose().

    void ref() { refThreadableWebSocketChannel(); }
    void deref() { derefThreadableWebSocketChannel(); }

protected:
    virtual ~ThreadableWebSocketChannel() { }
    virtual void refThreadableWebSocketChannel() = 0;
    virtual void derefThreadableWebSocketChannel() = 0;
};

} // namespace WebCore

#endif // ENABLE(WEB_SOCKETS)

#endif // ThreadableWebSocketChannel_h
