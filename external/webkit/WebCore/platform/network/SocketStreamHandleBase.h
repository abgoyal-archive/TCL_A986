

#ifndef SocketStreamHandleBase_h
#define SocketStreamHandleBase_h

#include "KURL.h"

#include <wtf/Vector.h>

namespace WebCore {

    class SocketStreamHandle;
    class SocketStreamHandleClient;

    class SocketStreamHandleBase {
    public:
        enum SocketStreamState { Connecting, Open, Closed };
        virtual ~SocketStreamHandleBase() { }
        SocketStreamState state() const;

        bool send(const char* data, int length);
        void close();
        int bufferedAmount() const { return m_buffer.size(); }

        SocketStreamHandleClient* client() const { return m_client; }
        void setClient(SocketStreamHandleClient*);

    protected:
        SocketStreamHandleBase(const KURL&, SocketStreamHandleClient*);

        bool sendPendingData();
        virtual int platformSend(const char* data, int length) = 0;
        virtual void platformClose() = 0;

        KURL m_url;
        SocketStreamHandleClient* m_client;
        Vector<char> m_buffer;
        SocketStreamState m_state;
    };

}  // namespace WebCore

#endif  // SocketStreamHandleBase_h
