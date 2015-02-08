

#include "config.h"

#include "CookieStorageWin.h"

#include <CFNetwork/CFHTTPCookiesPriv.h>
#include <WebKitSystemInterface/WebKitSystemInterface.h>
#include <wtf/RetainPtr.h>

namespace WebCore {

static RetainPtr<CFHTTPCookieStorageRef> s_cookieStorage;

CFHTTPCookieStorageRef currentCookieStorage()
{
    if (s_cookieStorage)
        return s_cookieStorage.get();
    return wkGetDefaultHTTPCookieStorage();
}

void setCurrentCookieStorage(CFHTTPCookieStorageRef cookieStorage)
{
    s_cookieStorage = cookieStorage;
}

}
