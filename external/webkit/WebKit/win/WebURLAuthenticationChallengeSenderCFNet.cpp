

#include "config.h"
#include "WebKitDLL.h"
#include <initguid.h>
#include "WebURLAuthenticationChallengeSender.h"

#include "COMPtr.h"
#include "WebKit.h"
#include "WebURLAuthenticationChallenge.h"
#include "WebURLCredential.h"

#pragma warning(push, 0)
#include <WebCore/AuthenticationClient.h>
#pragma warning(pop)

using namespace WebCore;

// IWebURLAuthenticationChallengeSender -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebURLAuthenticationChallengeSender::cancelAuthenticationChallenge(
        /* [in] */ IWebURLAuthenticationChallenge* challenge)
{
    COMPtr<WebURLAuthenticationChallenge> webChallenge(Query, challenge);
    if (!webChallenge)
        return E_FAIL;

    m_client->receivedCancellation(webChallenge->authenticationChallenge());
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebURLAuthenticationChallengeSender::continueWithoutCredentialForAuthenticationChallenge(
        /* [in] */ IWebURLAuthenticationChallenge* challenge)
{
    COMPtr<WebURLAuthenticationChallenge> webChallenge(Query, challenge);
    if (!webChallenge)
        return E_FAIL;

    m_client->receivedRequestToContinueWithoutCredential(webChallenge->authenticationChallenge());
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebURLAuthenticationChallengeSender::useCredential(
        /* [in] */ IWebURLCredential* credential, 
        /* [in] */ IWebURLAuthenticationChallenge* challenge)
{
    COMPtr<WebURLAuthenticationChallenge> webChallenge(Query, challenge);
    if (!webChallenge)
        return E_FAIL;
    
    COMPtr<WebURLCredential> webCredential;
    if (!credential || FAILED(credential->QueryInterface(__uuidof(WebURLCredential), (void**)&webCredential)))
        return E_FAIL;

    m_client->receivedCredential(webChallenge->authenticationChallenge(), webCredential->credential());
    return S_OK;
}
