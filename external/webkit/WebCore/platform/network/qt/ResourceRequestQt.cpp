

#include "config.h"
#include "ResourceRequest.h"

#include <qglobal.h>

#include <QNetworkRequest>
#include <QUrl>

namespace WebCore {

// Currently Qt allows three connections per host on symbian and six
// for everyone else. The limit can be found in qhttpnetworkconnection.cpp.
// To achieve the best result we want WebKit to schedule the jobs so we
// are using the limit as found in Qt. To allow Qt to fill its queue
// and prepare jobs we will schedule two more downloads.
unsigned initializeMaximumHTTPConnectionCountPerHost()
{
#ifdef Q_OS_SYMBIAN
    return 3 + 2;
#else
    return 6 + 2;
#endif
}

QNetworkRequest ResourceRequest::toNetworkRequest(QObject* originatingFrame) const
{
    QNetworkRequest request;
    request.setUrl(url());
#if QT_VERSION >= QT_VERSION_CHECK(4, 6, 0)
    request.setOriginatingObject(originatingFrame);
#endif

    const HTTPHeaderMap &headers = httpHeaderFields();
    for (HTTPHeaderMap::const_iterator it = headers.begin(), end = headers.end();
         it != end; ++it) {
        QByteArray name = QString(it->first).toAscii();
        QByteArray value = QString(it->second).toAscii();
        // QNetworkRequest::setRawHeader() would remove the header if the value is null
        // Make sure to set an empty header instead of null header.
        if (!value.isNull())
            request.setRawHeader(name, value);
        else
            request.setRawHeader(name, "");
    }

    switch (cachePolicy()) {
    case ReloadIgnoringCacheData:
        request.setAttribute(QNetworkRequest::CacheLoadControlAttribute, QNetworkRequest::AlwaysNetwork);
        break;
    case ReturnCacheDataElseLoad:
        request.setAttribute(QNetworkRequest::CacheLoadControlAttribute, QNetworkRequest::PreferCache);
        break;
    case ReturnCacheDataDontLoad:
        request.setAttribute(QNetworkRequest::CacheLoadControlAttribute, QNetworkRequest::AlwaysCache);
        break;
    case UseProtocolCachePolicy:
        // QNetworkRequest::PreferNetwork
    default:
        break;
    }

    return request;
}

}

