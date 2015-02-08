

#include "config.h"
#include "qwebdatabase.h"

#include "qwebdatabase_p.h"
#include "qwebsecurityorigin.h"
#include "qwebsecurityorigin_p.h"
#include "DatabaseDetails.h"
#include "DatabaseTracker.h"

using namespace WebCore;


QWebDatabase::QWebDatabase(const QWebDatabase& other)
    : d(other.d)
{
}

QWebDatabase& QWebDatabase::operator=(const QWebDatabase& other)
{
    d = other.d;
    return *this;
}

QString QWebDatabase::name() const
{
    return d->name;
}

QString QWebDatabase::displayName() const
{
#if ENABLE(DATABASE)
    DatabaseDetails details = DatabaseTracker::tracker().detailsForNameAndOrigin(d->name, d->origin.get());
    return details.displayName();
#else
    return QString();
#endif
}

qint64 QWebDatabase::expectedSize() const
{
#if ENABLE(DATABASE)
    DatabaseDetails details = DatabaseTracker::tracker().detailsForNameAndOrigin(d->name, d->origin.get());
    return details.expectedUsage();
#else
    return 0;
#endif
}

qint64 QWebDatabase::size() const
{
#if ENABLE(DATABASE)
    DatabaseDetails details = DatabaseTracker::tracker().detailsForNameAndOrigin(d->name, d->origin.get());
    return details.currentUsage();
#else
    return 0;
#endif
}

QWebDatabase::QWebDatabase(QWebDatabasePrivate* priv)
{
    d = priv;
}

QString QWebDatabase::fileName() const
{
#if ENABLE(DATABASE)
    return DatabaseTracker::tracker().fullPathForDatabase(d->origin.get(), d->name, false);
#else
    return QString();
#endif
}

QWebSecurityOrigin QWebDatabase::origin() const
{
    QWebSecurityOriginPrivate* priv = new QWebSecurityOriginPrivate(d->origin.get());
    QWebSecurityOrigin origin(priv);
    return origin;
}

void QWebDatabase::removeDatabase(const QWebDatabase& db)
{
#if ENABLE(DATABASE)
    DatabaseTracker::tracker().deleteDatabase(db.d->origin.get(), db.d->name);
#endif
}

void QWebDatabase::removeAllDatabases()
{
#if ENABLE(DATABASE)
    DatabaseTracker::tracker().deleteAllDatabases();
#endif
}

QWebDatabase::~QWebDatabase()
{
}

