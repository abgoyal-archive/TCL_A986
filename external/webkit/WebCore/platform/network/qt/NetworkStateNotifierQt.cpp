

#include "config.h"
#include "NetworkStateNotifier.h"

#include "NetworkStateNotifierPrivate.h"
#include "qnetworkconfigmanager.h"

using namespace QtMobility;

namespace WebCore {

NetworkStateNotifierPrivate::NetworkStateNotifierPrivate(NetworkStateNotifier* notifier)
    : m_configurationManager(new QNetworkConfigurationManager())
    , m_online(m_configurationManager->isOnline())
    , m_networkAccessAllowed(true)
    , m_notifier(notifier)
{
    Q_ASSERT(notifier);
    connect(m_configurationManager, SIGNAL(onlineStateChanged(bool)), this, SLOT(onlineStateChanged(bool)));
}

void NetworkStateNotifierPrivate::onlineStateChanged(bool isOnline)
{
    if (m_online == isOnline)
        return;

    m_online = isOnline;
    if (m_networkAccessAllowed)
        m_notifier->updateState();
}

void NetworkStateNotifierPrivate::networkAccessPermissionChanged(bool isAllowed)
{
    if (isAllowed == m_networkAccessAllowed)
        return;

    m_networkAccessAllowed = isAllowed;
    if (m_online)
        m_notifier->updateState();
}

NetworkStateNotifierPrivate::~NetworkStateNotifierPrivate()
{
    delete m_configurationManager;
}

void NetworkStateNotifier::updateState()
{
    if (m_isOnLine == (p->m_online && p->m_networkAccessAllowed))
        return;

    m_isOnLine = p->m_online && p->m_networkAccessAllowed;
    if (m_networkStateChangedFunction)
        m_networkStateChangedFunction();
}

NetworkStateNotifier::NetworkStateNotifier()
    : m_isOnLine(true)
    , m_networkStateChangedFunction(0)
{
    p = new NetworkStateNotifierPrivate(this);
    m_isOnLine = p->m_online && p->m_networkAccessAllowed;
}

void NetworkStateNotifier::setNetworkAccessAllowed(bool isAllowed)
{
    p->networkAccessPermissionChanged(isAllowed);
}

} // namespace WebCore

#include "moc_NetworkStateNotifierPrivate.cpp"
