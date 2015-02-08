

#include "config.h"
#include "SocketStreamHandleBase.h"

#include "SocketStreamHandle.h"
#include "SocketStreamHandleClient.h"

namespace WebCore {

const unsigned int bufferSize = 100 * 1024 * 1024;

SocketStreamHandleBase::SocketStreamHandleBase(const KURL& url, SocketStreamHandleClient* client)
    : m_url(url)
    , m_client(client)
    , m_state(Connecting)
{
}

SocketStreamHandleBase::SocketStreamState SocketStreamHandleBase::state() const
{
    return m_state;
}

bool SocketStreamHandleBase::send(const char* data, int length)
{
    if (m_state == Connecting)
        return false;
    if (!m_buffer.isEmpty()) {
        if (m_buffer.size() + length > bufferSize) {
            // FIXME: report error to indicate that buffer has no more space.
            return false;
        }
        m_buffer.append(data, length);
        return true;
    }
    int bytesWritten = 0;
    if (m_state == Open)
        bytesWritten = platformSend(data, length);
    if (bytesWritten < 0)
        return false;
    if (m_buffer.size() + length - bytesWritten > bufferSize) {
        // FIXME: report error to indicate that buffer has no more space.
        return false;
    }
    if (bytesWritten < length)
        m_buffer.append(data + bytesWritten, length - bytesWritten);
    return true;
}

void SocketStreamHandleBase::close()
{
    RefPtr<SocketStreamHandle> protect(static_cast<SocketStreamHandle*>(this)); // platformClose calls the client, which may make the handle get deallocated immediately.

    platformClose();
    m_state = Closed;
}

void SocketStreamHandleBase::setClient(SocketStreamHandleClient* client)
{
    ASSERT(!client || (!m_client && m_state == Connecting));
    m_client = client;
}

bool SocketStreamHandleBase::sendPendingData()
{
    if (m_state != Open)
        return false;
    if (m_buffer.isEmpty())
        return false;
    int bytesWritten = platformSend(m_buffer.data(), m_buffer.size());
    if (bytesWritten <= 0)
        return false;
    Vector<char> remainingData;
    ASSERT(m_buffer.size() - bytesWritten <= bufferSize);
    remainingData.append(m_buffer.data() + bytesWritten, m_buffer.size() - bytesWritten);
    m_buffer.swap(remainingData);
    return true;
}

}  // namespace WebCore
