

#include "config.h"
#include "WebKitDLL.h"
#include <initguid.h>
#include "WebURLAuthenticationChallengeSender.h"

#include "COMPtr.h"
#include "NotImplemented.h"
#include "WebKit.h"
#include "WebURLAuthenticationChallenge.h"
#include "WebURLCredential.h"

#pragma warning(push, 0)
#include <WebCore/ResourceHandle.h>
#pragma warning(pop)

using namespace WebCore;

// IWebURLAuthenticationChallengeSender -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebURLAuthenticationChallengeSender::cancelAuthenticationChallenge(
        /* [in] */ IWebURLAuthenticationChallenge* challenge)
{
    notImplemented();
    return E_FAIL;
}

HRESULT STDMETHODCALLTYPE WebURLAuthenticationChallengeSender::continueWithoutCredentialForAuthenticationChallenge(
        /* [in] */ IWebURLAuthenticationChallenge* challenge)
{
    notImplemented();
    return E_FAIL;
}

HRESULT STDMETHODCALLTYPE WebURLAuthenticationChallengeSender::useCredential(
        /* [in] */ IWebURLCredential* credential, 
        /* [in] */ IWebURLAuthenticationChallenge* challenge)
{
    notImplemented();
    return E_FAIL;
}
