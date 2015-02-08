

#ifndef _WEBSECURITYORIGIN_H_
#define _WEBSECURITYORIGIN_H_

#include <QtCore/qurl.h>
#include <QtCore/qshareddata.h>

#include "qwebkitglobal.h"

namespace WebCore {
    class SecurityOrigin;
    class ChromeClientQt;
}

class QWebSecurityOriginPrivate;
class QWebDatabase;
class QWebFrame;

class QWEBKIT_EXPORT QWebSecurityOrigin {
public:
    static QList<QWebSecurityOrigin> allOrigins();
    static void addLocalScheme(const QString& scheme);
    static void removeLocalScheme(const QString& scheme);
    static QStringList localSchemes();

    ~QWebSecurityOrigin();

    QString scheme() const;
    QString host() const;
    int port() const;

    qint64 databaseUsage() const;
    qint64 databaseQuota() const;

    void setDatabaseQuota(qint64 quota);

    QList<QWebDatabase> databases() const;

    QWebSecurityOrigin(const QWebSecurityOrigin& other);
    QWebSecurityOrigin &operator=(const QWebSecurityOrigin& other);
private:
    friend class QWebDatabase;
    friend class QWebFrame;
    friend class WebCore::ChromeClientQt;
    QWebSecurityOrigin(QWebSecurityOriginPrivate* priv);

private:
    QExplicitlySharedDataPointer<QWebSecurityOriginPrivate> d;
};

#endif
