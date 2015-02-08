

#include "config.h"
#include "WebKitDLL.h"
#include "WebCookieManager.h"

#include <CFNetwork/CFHTTPCookiesPriv.h>
#include <WebCore/CookieStorageWin.h>

using namespace WebCore;

// IWebCookieManager -------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebCookieManager::cookieStorage( 
    /* [retval][out] */ CFHTTPCookieStorageRef* storage)
{
   if (!storage)
        return E_POINTER;

    *storage = currentCookieStorage();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebCookieManager::setCookieStorage( 
    /* [in] */ CFHTTPCookieStorageRef storage)
{
    setCurrentCookieStorage(storage);
    return S_OK;
}
