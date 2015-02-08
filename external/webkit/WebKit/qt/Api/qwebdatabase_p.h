
#ifndef _WEBDATABASE_P_H_
#define _WEBDATABASE_P_H_

#include <QString>
#include <QtCore/qshareddata.h>

#include "PlatformString.h"
#include "SecurityOrigin.h"
#include "RefPtr.h"


class QWebDatabasePrivate : public QSharedData {
public:
    WebCore::String name;
    WTF::RefPtr<WebCore::SecurityOrigin> origin;
};

#endif

