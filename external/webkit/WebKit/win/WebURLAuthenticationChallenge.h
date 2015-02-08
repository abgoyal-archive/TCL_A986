

#ifndef WebURLAuthenticationChallenge_h
#define WebURLAuthenticationChallenge_h

#include "WebKit.h"

#pragma warning(push, 0)
#include <WebCore/AuthenticationChallenge.h>
#include <WebCore/COMPtr.h>
#pragma warning(pop)

class DECLSPEC_UUID("FD3B2381-0BB6-4B59-AF09-0E599C8901CF") WebURLAuthenticationChallenge : public IWebURLAuthenticationChallenge {
public:
    static WebURLAuthenticationChallenge* createInstance(const WebCore::AuthenticationChallenge&);
    static WebURLAuthenticationChallenge* createInstance(const WebCore::AuthenticationChallenge&, IWebURLAuthenticationChallengeSender*);
private:
    WebURLAuthenticationChallenge(const WebCore::AuthenticationChallenge&, IWebURLAuthenticationChallengeSender*);
    ~WebURLAuthenticationChallenge();
public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebURLAuthenticationChallenge
    virtual HRESULT STDMETHODCALLTYPE initWithProtectionSpace(
        /* [in] */ IWebURLProtectionSpace* space, 
        /* [in] */ IWebURLCredential* proposedCredential, 
        /* [in] */ int previousFailureCount, 
        /* [in] */ IWebURLResponse* failureResponse, 
        /* [in] */ IWebError* error, 
        /* [in] */ IWebURLAuthenticationChallengeSender* sender);

    virtual HRESULT STDMETHODCALLTYPE initWithAuthenticationChallenge(
        /* [in] */ IWebURLAuthenticationChallenge* challenge, 
        /* [in] */ IWebURLAuthenticationChallengeSender* sender);

    virtual HRESULT STDMETHODCALLTYPE error(
        /* [out, retval] */ IWebError** result);

    virtual HRESULT STDMETHODCALLTYPE failureResponse(
        /* [out, retval] */ IWebURLResponse** result);

    virtual HRESULT STDMETHODCALLTYPE previousFailureCount(
        /* [out, retval] */ UINT* result);

    virtual HRESULT STDMETHODCALLTYPE proposedCredential(
        /* [out, retval] */ IWebURLCredential** result);

    virtual HRESULT STDMETHODCALLTYPE protectionSpace(
        /* [out, retval] */ IWebURLProtectionSpace** result);

    virtual HRESULT STDMETHODCALLTYPE sender(
        /* [out, retval] */ IWebURLAuthenticationChallengeSender** sender);

    // WebURLAuthenticationChallenge
    const WebCore::AuthenticationChallenge& authenticationChallenge() const;

protected:
    ULONG m_refCount;

    WebCore::AuthenticationChallenge m_authenticationChallenge;
    COMPtr<IWebURLAuthenticationChallengeSender> m_sender;
};


#endif
