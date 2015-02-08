

#ifndef ThreadableWebSocketChannelClientWrapper_h
#define ThreadableWebSocketChannelClientWrapper_h

#if ENABLE(WEB_SOCKETS)

#include "WebSocketChannelClient.h"
#include <wtf/PassRefPtr.h>
#include <wtf/Threading.h>

namespace WebCore {

class String;

class ThreadableWebSocketChannelClientWrapper : public ThreadSafeShared<ThreadableWebSocketChannelClientWrapper> {
public:
    static PassRefPtr<ThreadableWebSocketChannelClientWrapper> create(WebSocketChannelClient* client)
    {
        return adoptRef(new ThreadableWebSocketChannelClientWrapper(client));
    }

    void clearSyncMethodDone()
    {
        m_syncMethodDone = false;
    }
    void setSyncMethodDone()
    {
        m_syncMethodDone = true;
    }

    bool syncMethodDone() const
    {
        return m_syncMethodDone;
    }

    bool sent() const
    {
        return m_sent;
    }
    void setSent(bool sent)
    {
        m_sent = sent;
        m_syncMethodDone = true;
    }

    unsigned long bufferedAmount() const
    {
        return m_bufferedAmount;
    }
    void setBufferedAmount(unsigned long bufferedAmount)
    {
        m_bufferedAmount = bufferedAmount;
        m_syncMethodDone = true;
    }

    void clearClient()
    {
        m_client = 0;
    }

    void didConnect()
    {
        if (m_client)
            m_client->didConnect();
    }

    void didReceiveMessage(const String& msg)
    {
        if (m_client)
            m_client->didReceiveMessage(msg);
    }

    void didClose(unsigned long unhandledBufferedAmount)
    {
        if (m_client)
            m_client->didClose(unhandledBufferedAmount);
    }

protected:
    ThreadableWebSocketChannelClientWrapper(WebSocketChannelClient* client)
        : m_client(client)
        , m_syncMethodDone(false)
        , m_sent(false)
        , m_bufferedAmount(0)
    {
    }

    WebSocketChannelClient* m_client;
    bool m_syncMethodDone;
    bool m_sent;
    unsigned long m_bufferedAmount;
};

} // namespace WebCore

#endif // ENABLE(WEB_SOCKETS)

#endif // ThreadableWebSocketChannelClientWrapper_h
