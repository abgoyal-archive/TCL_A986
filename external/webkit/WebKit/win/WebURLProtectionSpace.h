

#ifndef WebURLProtectionSpace_h
#define WebURLProtectionSpace_h

#include "WebKit.h"

#pragma warning(push, 0)
#include <WebCore/ProtectionSpace.h>
#pragma warning(pop)

class WebURLProtectionSpace : public IWebURLProtectionSpace
{
public:
    static WebURLProtectionSpace* createInstance();
    static WebURLProtectionSpace* createInstance(const WebCore::ProtectionSpace&);
private:
    WebURLProtectionSpace(const WebCore::ProtectionSpace&);
    ~WebURLProtectionSpace();
public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebURLProtectionSpace
    virtual HRESULT STDMETHODCALLTYPE authenticationMethod(
        /* [out, retval] */ BSTR* result);

    virtual HRESULT STDMETHODCALLTYPE host(
        /* [out, retval] */ BSTR* result);

    virtual HRESULT STDMETHODCALLTYPE initWithHost(
        /* [in] */ BSTR host, 
        /* [in] */ int port, 
        /* [in] */ BSTR protocol, 
        /* [in] */ BSTR realm, 
        /* [in] */ BSTR authenticationMethod);

    virtual HRESULT STDMETHODCALLTYPE initWithProxyHost(
        /* [in] */ BSTR host, 
        /* [in] */ int port, 
        /* [in] */ BSTR proxyType, 
        /* [in] */ BSTR realm, 
        /* [in] */ BSTR authenticationMethod);

    virtual HRESULT STDMETHODCALLTYPE isProxy(
        /* [out, retval] */ BOOL* result);

    virtual HRESULT STDMETHODCALLTYPE port(
        /* [out, retval] */ int* result);

    virtual HRESULT STDMETHODCALLTYPE protocol(
        /* [out, retval] */ BSTR* result);

    virtual HRESULT STDMETHODCALLTYPE proxyType(
        /* [out, retval] */ BSTR* result);

    virtual HRESULT STDMETHODCALLTYPE realm(
        /* [out, retval] */ BSTR* result);

    virtual HRESULT STDMETHODCALLTYPE receivesCredentialSecurely(
        /* [out, retval] */ BOOL* result);

    // WebURLProtectionSpace
    const WebCore::ProtectionSpace& protectionSpace() const;

protected:
    ULONG m_refCount;

    WebCore::ProtectionSpace m_protectionSpace;
};


#endif
