

#include "config.h"
#include "SocketStreamHandle.h"

#include "KURL.h"
#include "Logging.h"
#include "NotImplemented.h"
#include "SocketStreamHandleClient.h"

namespace WebCore {

SocketStreamHandle::SocketStreamHandle(const KURL& url, SocketStreamHandleClient* client)
    : SocketStreamHandleBase(url, client)
{
    LOG(Network, "SocketStreamHandle %p new client %p", this, m_client);
    notImplemented();
}

SocketStreamHandle::~SocketStreamHandle()
{
    LOG(Network, "SocketStreamHandle %p delete", this);
    setClient(0);
    notImplemented();
}

int SocketStreamHandle::platformSend(const char*, int)
{
    LOG(Network, "SocketStreamHandle %p platformSend", this);
    notImplemented();
    return 0;
}

void SocketStreamHandle::platformClose()
{
    LOG(Network, "SocketStreamHandle %p platformClose", this);
    notImplemented();
}

void SocketStreamHandle::didReceiveAuthenticationChallenge(const AuthenticationChallenge&)
{
    notImplemented();
}

void SocketStreamHandle::receivedCredential(const AuthenticationChallenge&, const Credential&)
{
    notImplemented();
}

void SocketStreamHandle::receivedRequestToContinueWithoutCredential(const AuthenticationChallenge&)
{
    notImplemented();
}

void SocketStreamHandle::receivedCancellation(const AuthenticationChallenge&)
{
    notImplemented();
}

}  // namespace WebCore
