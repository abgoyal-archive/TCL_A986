

#ifndef WebURLAuthenticationChallengeSender_h
#define WebURLAuthenticationChallengeSender_h

#include "WebKit.h"

#include <wtf/Forward.h>
#include <wtf/RefPtr.h>

namespace WebCore {
    class AuthenticationClient;
}

class DECLSPEC_UUID("5CACD637-F82F-491F-947A-5DCA38AA0FEA") WebURLAuthenticationChallengeSender
    : public IWebURLAuthenticationChallengeSender
{
public:
    static WebURLAuthenticationChallengeSender* createInstance(PassRefPtr<WebCore::AuthenticationClient>);
private:
    WebURLAuthenticationChallengeSender(PassRefPtr<WebCore::AuthenticationClient>);
    ~WebURLAuthenticationChallengeSender();
public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebURLAuthenticationChallengeSender
    virtual HRESULT STDMETHODCALLTYPE cancelAuthenticationChallenge(
        /* [in] */ IWebURLAuthenticationChallenge* challenge);

    virtual HRESULT STDMETHODCALLTYPE continueWithoutCredentialForAuthenticationChallenge(
        /* [in] */ IWebURLAuthenticationChallenge* challenge);

    virtual HRESULT STDMETHODCALLTYPE useCredential(
        /* [in] */ IWebURLCredential* credential, 
        /* [in] */ IWebURLAuthenticationChallenge* challenge);

    WebCore::AuthenticationClient* authenticationClient() const;

private:
    ULONG m_refCount;

    RefPtr<WebCore::AuthenticationClient> m_client;
};

#endif
