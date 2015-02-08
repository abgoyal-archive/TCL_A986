

#include "config.h"
#include "WebKitDLL.h"
#include "WebCookieManager.h"

#include "NotImplemented.h"

using namespace WebCore;

// IWebCookieManager -------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebCookieManager::cookieStorage( 
    /* [retval][out] */ CFHTTPCookieStorageRef* storage)
{
   notImplemented();
   return E_FAIL;
}

HRESULT STDMETHODCALLTYPE WebCookieManager::setCookieStorage( 
    /* [in] */ CFHTTPCookieStorageRef storage)
{
   notImplemented();
   return E_FAIL;
}
