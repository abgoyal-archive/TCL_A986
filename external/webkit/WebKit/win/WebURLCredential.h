

#ifndef WebURLCredential_h
#define WebURLCredential_h

#include "WebKit.h"

#pragma warning(push, 0)
#include <WebCore/Credential.h>
#pragma warning(pop)

class WebURLCredential : public IWebURLCredential
{
public:
    static WebURLCredential* createInstance();
    static WebURLCredential* createInstance(const WebCore::Credential&);
private:
    WebURLCredential(const WebCore::Credential&);
    ~WebURLCredential();
public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebURLCredential
    virtual HRESULT STDMETHODCALLTYPE hasPassword(
        /* [out, retval] */ BOOL* result);

    virtual HRESULT STDMETHODCALLTYPE initWithUser(
        /* [in] */ BSTR user, 
        /* [in] */ BSTR password, 
        /* [in] */ WebURLCredentialPersistence persistence);

    virtual HRESULT STDMETHODCALLTYPE password(
        /* [out, retval] */ BSTR* password);

    virtual HRESULT STDMETHODCALLTYPE persistence(
        /* [out, retval] */ WebURLCredentialPersistence* result);

    virtual HRESULT STDMETHODCALLTYPE user(
        /* [out, retval] */ BSTR* result);

    // WebURLCredential
    const WebCore::Credential& credential() const;

protected:
    ULONG m_refCount;

    WebCore::Credential m_credential;
};


#endif
