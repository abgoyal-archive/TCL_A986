

#ifndef SocketStreamHandlePrivate_h
#define SocketStreamHandlePrivate_h

#include "SocketStreamHandleBase.h"

#include <QSslSocket>
#include <QTcpSocket>

namespace WebCore {

class AuthenticationChallenge;
class Credential;
class SocketStreamHandleClient;
class SocketStreamHandlePrivate;

class SocketStreamHandlePrivate : public QObject {
    Q_OBJECT
public:
    SocketStreamHandlePrivate(SocketStreamHandle*, const KURL&);
    ~SocketStreamHandlePrivate();

public slots:
    void socketConnected();
    void socketReadyRead();
    int send(const char* data, int len);
    void close();
    void socketSentdata();
    void socketClosed();
    void socketError(QAbstractSocket::SocketError);
    void socketClosedCallback();
    void socketErrorCallback(int);
#ifndef QT_NO_OPENSSL
    void socketSslErrors(const QList<QSslError>&);
#endif
public:
    QTcpSocket* m_socket;
    SocketStreamHandle* m_streamHandle;
};

}

#endif
