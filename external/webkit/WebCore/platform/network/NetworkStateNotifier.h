

#ifndef NetworkStateNotifier_h
#define NetworkStateNotifier_h

#include <wtf/Noncopyable.h>
#if PLATFORM(ANDROID)
// TODO: Upstream to webkit.org
#include "Connection.h"
#endif

#if PLATFORM(MAC)

#include <wtf/RetainPtr.h>
#include "Timer.h"

typedef const struct __CFArray * CFArrayRef;
typedef const struct __SCDynamicStore * SCDynamicStoreRef;

#elif PLATFORM(CHROMIUM)

#include "NetworkStateNotifierPrivate.h"

#elif PLATFORM(WIN)

#include <windows.h>

#endif

namespace WebCore {

#if (PLATFORM(QT) && ENABLE(QT_BEARER))
class NetworkStateNotifierPrivate;
#endif

class NetworkStateNotifier : public Noncopyable {
public:
    NetworkStateNotifier();
    void setNetworkStateChangedFunction(void (*)());
    
    bool onLine() const { return m_isOnLine; }

#if (PLATFORM(QT) && ENABLE(QT_BEARER))
    void setNetworkAccessAllowed(bool);
#endif

#if PLATFORM(ANDROID)
    // TODO: Upstream to webkit.org
    Connection::ConnectionType type() const { return m_type; }
#endif

private:    
    bool m_isOnLine;
#if PLATFORM(ANDROID)
    // TODO: Upstream to webkit.org
    Connection::ConnectionType m_type;
#endif
    void (*m_networkStateChangedFunction)();

    void updateState();

#if PLATFORM(MAC)
    void networkStateChangeTimerFired(Timer<NetworkStateNotifier>*);

    static void dynamicStoreCallback(SCDynamicStoreRef, CFArrayRef changedKeys, void *info); 

    RetainPtr<SCDynamicStoreRef> m_store;
    Timer<NetworkStateNotifier> m_networkStateChangeTimer;

#elif PLATFORM(WIN)
    static void CALLBACK addrChangeCallback(void*, BOOLEAN timedOut);
    static void callAddressChanged(void*);
    void addressChanged();
    
    void registerForAddressChange();
    HANDLE m_waitHandle;
    OVERLAPPED m_overlapped;

#elif PLATFORM(CHROMIUM)
    NetworkStateNotifierPrivate p;

#elif PLATFORM(ANDROID)
public:
    void networkStateChange(bool online);
    // TODO: Upstream to webkit.org
    void networkTypeChange(Connection::ConnectionType type);

#elif PLATFORM(QT) && ENABLE(QT_BEARER)
    friend class NetworkStateNotifierPrivate;
    NetworkStateNotifierPrivate* p;
#endif
};

#if !PLATFORM(MAC) && !PLATFORM(WIN) && !PLATFORM(CHROMIUM) && !(PLATFORM(QT) && ENABLE(QT_BEARER))

inline NetworkStateNotifier::NetworkStateNotifier()
    : m_isOnLine(true)
#if PLATFORM(ANDROID)
    // TODO: Upstream to webkit.org
    , m_type(Connection::Unknown)
#endif
    , m_networkStateChangedFunction(0)
{    
}

inline void NetworkStateNotifier::updateState() { }

#endif

NetworkStateNotifier& networkStateNotifier();
    
};

#endif // NetworkStateNotifier_h
