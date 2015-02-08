

#ifndef ThreadableLoaderClientWrapper_h
#define ThreadableLoaderClientWrapper_h

#include "ThreadableLoaderClient.h"
#include <wtf/Noncopyable.h>
#include <wtf/PassRefPtr.h>
#include <wtf/Threading.h>

namespace WebCore {

    class ThreadableLoaderClientWrapper : public ThreadSafeShared<ThreadableLoaderClientWrapper> {
    public:
        static PassRefPtr<ThreadableLoaderClientWrapper> create(ThreadableLoaderClient* client)
        {
            return adoptRef(new ThreadableLoaderClientWrapper(client));
        }

        void clearClient()
        {
            m_done = true;
            m_client = 0;
        }

        bool done() const
        {
            return m_done;
        }

        void didSendData(unsigned long long bytesSent, unsigned long long totalBytesToBeSent)
        {
            if (m_client)
                m_client->didSendData(bytesSent, totalBytesToBeSent);
        }

        void didReceiveResponse(const ResourceResponse& response)
        {
            if (m_client)
                m_client->didReceiveResponse(response);
        }

        void didReceiveData(const char* data, int lengthReceived)
        {
            if (m_client)
                m_client->didReceiveData(data, lengthReceived);
        }

        void didFinishLoading(unsigned long identifier)
        {
            m_done = true;
            if (m_client)
                m_client->didFinishLoading(identifier);
        }

        void didFail(const ResourceError& error)
        {
            m_done = true;
            if (m_client)
                m_client->didFail(error);
        }

        void didFailRedirectCheck()
        {
            m_done = true;
            if (m_client)
                m_client->didFailRedirectCheck();
        }

        void didReceiveAuthenticationCancellation(const ResourceResponse& response)
        {
            if (m_client)
                m_client->didReceiveResponse(response);
        }

    protected:
        ThreadableLoaderClientWrapper(ThreadableLoaderClient* client)
            : m_client(client)
            , m_done(false)
        {
        }

        ThreadableLoaderClient* m_client;
        bool m_done;
    };

} // namespace WebCore

#endif // ThreadableLoaderClientWrapper_h
