

#include "config.h"
#include "NetworkStateNotifier.h"

#include <wtf/MainThread.h>
#include <wtf/Vector.h>

#include <winsock2.h>
#include <iphlpapi.h>

namespace WebCore {

void NetworkStateNotifier::updateState()
{
    // Assume that we're online until proven otherwise.
    m_isOnLine = true;
    
    Vector<char> buffer;
    DWORD size = 0;

    if (::GetAdaptersAddresses(AF_UNSPEC, 0, 0, 0, &size) != ERROR_BUFFER_OVERFLOW)
        return;

    buffer.resize(size);
    PIP_ADAPTER_ADDRESSES addresses = reinterpret_cast<PIP_ADAPTER_ADDRESSES>(buffer.data());

    if (::GetAdaptersAddresses(AF_UNSPEC, 0, 0, addresses, &size) != ERROR_SUCCESS) {
        // We couldn't determine whether we're online or not, so assume that we are.
        return;
    }

    for (; addresses; addresses = addresses->Next) {
        if (addresses->IfType == MIB_IF_TYPE_LOOPBACK)
            continue;

        if (addresses->OperStatus != IfOperStatusUp)
            continue;

        // We found an interface that was up.
        return;
    }
    
    // We didn't find any valid interfaces, so we must be offline.
    m_isOnLine = false;
}

void NetworkStateNotifier::addressChanged()
{
    bool oldOnLine = m_isOnLine;
    
    updateState();

    if (m_isOnLine == oldOnLine)
        return;

    if (m_networkStateChangedFunction)
        m_networkStateChangedFunction();
}

void NetworkStateNotifier::callAddressChanged(void* context)
{
    static_cast<NetworkStateNotifier*>(context)->addressChanged();
}

void CALLBACK NetworkStateNotifier::addrChangeCallback(void* context, BOOLEAN timedOut)
{
    callOnMainThread(callAddressChanged, context);
}

void NetworkStateNotifier::registerForAddressChange()
{
    HANDLE handle;
    ::NotifyAddrChange(&handle, &m_overlapped);
}

NetworkStateNotifier::NetworkStateNotifier()
    : m_isOnLine(false)
    , m_networkStateChangedFunction(0)
{
    updateState();

    memset(&m_overlapped, 0, sizeof(m_overlapped));

    m_overlapped.hEvent = ::CreateEvent(0, false, false, 0);

    ::RegisterWaitForSingleObject(&m_waitHandle, m_overlapped.hEvent, addrChangeCallback, this, INFINITE, 0);

    registerForAddressChange();
}

}
