

#ifndef NetworkStateNotifierPrivate_h
#define NetworkStateNotifierPrivate_h

#include <QObject>

namespace QtMobility {
class QNetworkConfigurationManager;
}

namespace WebCore {

class NetworkStateNotifier;

class NetworkStateNotifierPrivate : public QObject {
    Q_OBJECT
public:
    NetworkStateNotifierPrivate(NetworkStateNotifier* notifier);
    ~NetworkStateNotifierPrivate();
public slots:
    void onlineStateChanged(bool);
    void networkAccessPermissionChanged(bool);

public:
    QtMobility::QNetworkConfigurationManager* m_configurationManager;
    bool m_online;
    bool m_networkAccessAllowed;
    NetworkStateNotifier* m_notifier;
};

} // namespace WebCore

#endif
