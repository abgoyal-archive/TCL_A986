
#ifndef _WEBDATABASE_H_
#define _WEBDATABASE_H_

#include <QtCore/qstring.h>
#include <QtCore/qshareddata.h>

#include "qwebkitglobal.h"

namespace WebCore {
    class DatabaseDetails;
}

class QWebDatabasePrivate;
class QWebSecurityOrigin;

class QWEBKIT_EXPORT QWebDatabase {
public:
    QWebDatabase(const QWebDatabase& other);
    QWebDatabase &operator=(const QWebDatabase& other);
    ~QWebDatabase();

    QString name() const;
    QString displayName() const;
    qint64 expectedSize() const;
    qint64 size() const;
    QString fileName() const;
    QWebSecurityOrigin origin() const;

    static void removeDatabase(const QWebDatabase&);
    static void removeAllDatabases();

private:
    QWebDatabase(QWebDatabasePrivate* priv);
    friend class QWebSecurityOrigin;

private:
    QExplicitlySharedDataPointer<QWebDatabasePrivate> d;
};

#endif

