

#include "config.h"
#include "qwebsecurityorigin.h"
#include "qwebsecurityorigin_p.h"
#include "qwebdatabase.h"
#include "qwebdatabase_p.h"

#include "DatabaseTracker.h"
#include "KURL.h"
#include "SecurityOrigin.h"
#include <QStringList>

using namespace WebCore;

void QWEBKIT_EXPORT qt_drt_whiteListAccessFromOrigin(const QString& sourceOrigin, const QString& destinationProtocol, const QString& destinationHost, bool allowDestinationSubdomains)
{
    SecurityOrigin::whiteListAccessFromOrigin(*SecurityOrigin::createFromString(sourceOrigin), destinationProtocol, destinationHost, allowDestinationSubdomains);
}

void QWEBKIT_EXPORT qt_drt_resetOriginAccessWhiteLists()
{
    SecurityOrigin::resetOriginAccessWhiteLists();
}

void QWEBKIT_EXPORT qt_drt_setDomainRelaxationForbiddenForURLScheme(bool forbidden, const QString& scheme)
{
    SecurityOrigin::setDomainRelaxationForbiddenForURLScheme(forbidden, scheme);
}


QWebSecurityOrigin::QWebSecurityOrigin(const QWebSecurityOrigin& other) : d(other.d)
{
}

QWebSecurityOrigin& QWebSecurityOrigin::operator=(const QWebSecurityOrigin& other)
{
    d = other.d;
    return *this;
}

QString QWebSecurityOrigin::scheme() const
{
    return d->origin->protocol();
}

QString QWebSecurityOrigin::host() const
{
    return d->origin->host();
}

int QWebSecurityOrigin::port() const
{
    return d->origin->port();
}

qint64 QWebSecurityOrigin::databaseUsage() const
{
#if ENABLE(DATABASE)
    return DatabaseTracker::tracker().usageForOrigin(d->origin.get());
#else
    return 0;
#endif
}

qint64 QWebSecurityOrigin::databaseQuota() const
{
#if ENABLE(DATABASE)
    return DatabaseTracker::tracker().quotaForOrigin(d->origin.get());
#else
    return 0;
#endif
}

void QWebSecurityOrigin::setDatabaseQuota(qint64 quota)
{
#if ENABLE(DATABASE)
    DatabaseTracker::tracker().setQuota(d->origin.get(), quota);
#endif
}

QWebSecurityOrigin::~QWebSecurityOrigin()
{
}

QWebSecurityOrigin::QWebSecurityOrigin(QWebSecurityOriginPrivate* priv)
{
    d = priv;
}

QList<QWebSecurityOrigin> QWebSecurityOrigin::allOrigins()
{
    QList<QWebSecurityOrigin> webOrigins;

#if ENABLE(DATABASE)
    Vector<RefPtr<SecurityOrigin> > coreOrigins;
    DatabaseTracker::tracker().origins(coreOrigins);

    for (unsigned i = 0; i < coreOrigins.size(); ++i) {
        QWebSecurityOriginPrivate* priv = new QWebSecurityOriginPrivate(coreOrigins[i].get());
        webOrigins.append(priv);
    }
#endif

    return webOrigins;
}

QList<QWebDatabase> QWebSecurityOrigin::databases() const
{
    QList<QWebDatabase> databases;

#if ENABLE(DATABASE)
    Vector<String> nameVector;

    if (!DatabaseTracker::tracker().databaseNamesForOrigin(d->origin.get(), nameVector))
        return databases;
    for (unsigned i = 0; i < nameVector.size(); ++i) {
        QWebDatabasePrivate* priv = new QWebDatabasePrivate();
        priv->name = nameVector[i];
        priv->origin = this->d->origin;
        QWebDatabase webDatabase(priv);
        databases.append(webDatabase);
    }
#endif

    return databases;
}

void QWebSecurityOrigin::addLocalScheme(const QString& scheme)
{
    SecurityOrigin::registerURLSchemeAsLocal(scheme);
}

void QWebSecurityOrigin::removeLocalScheme(const QString& scheme)
{
    SecurityOrigin::removeURLSchemeRegisteredAsLocal(scheme);
}

QStringList QWebSecurityOrigin::localSchemes()
{
    QStringList list;
    const URLSchemesMap& map = SecurityOrigin::localURLSchemes();
    URLSchemesMap::const_iterator end = map.end();
    for (URLSchemesMap::const_iterator i = map.begin(); i != end; ++i) {
        const QString scheme = *i;
        list.append(scheme);
    }
    return list;
}
