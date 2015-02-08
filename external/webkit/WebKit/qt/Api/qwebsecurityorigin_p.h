

#ifndef _WEBSECURITYORIGIN_P_H_
#define _WEBSECURITYORIGIN_P_H_

#include "SecurityOrigin.h"
#include "RefPtr.h"

class QWebSecurityOriginPrivate : public QSharedData {
public:
    QWebSecurityOriginPrivate(WebCore::SecurityOrigin* o)
    {
        Q_ASSERT(o);
        origin = o;
    }
    ~QWebSecurityOriginPrivate()
    {
    }
    WTF::RefPtr<WebCore::SecurityOrigin> origin;
};

#endif
