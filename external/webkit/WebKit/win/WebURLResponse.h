

#ifndef WebURLResponse_H
#define WebURLResponse_H

#include "WebKit.h"

#pragma warning(push, 0)
#include <WebCore/ResourceResponse.h>
#pragma warning(pop)

class WebURLResponse : public IWebHTTPURLResponse, IWebURLResponsePrivate
{
public:
    static WebURLResponse* createInstance();
    static WebURLResponse* createInstance(const WebCore::ResourceResponse& response);
protected:
    WebURLResponse();
    ~WebURLResponse();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebURLResponse
    virtual HRESULT STDMETHODCALLTYPE expectedContentLength( 
        /* [retval][out] */ long long *result);
    
    virtual HRESULT STDMETHODCALLTYPE initWithURL( 
        /* [in] */ BSTR url,
        /* [in] */ BSTR mimeType,
        /* [in] */ int expectedContentLength,
        /* [in] */ BSTR textEncodingName);
    
    virtual HRESULT STDMETHODCALLTYPE MIMEType( 
        /* [retval][out] */ BSTR *result);
    
    virtual HRESULT STDMETHODCALLTYPE suggestedFilename( 
        /* [retval][out] */ BSTR *result);
    
    virtual HRESULT STDMETHODCALLTYPE textEncodingName( 
        /* [retval][out] */ BSTR *result);
    
    virtual HRESULT STDMETHODCALLTYPE URL( 
        /* [retval][out] */ BSTR *result);

    // IWebHTTPURLResponse
    virtual HRESULT STDMETHODCALLTYPE allHeaderFields( 
        /* [retval][out] */ IPropertyBag **headerFields);
    
    virtual HRESULT STDMETHODCALLTYPE localizedStringForStatusCode( 
        /* [in] */ int statusCode,
        /* [retval][out] */ BSTR *statusString);
    
    virtual HRESULT STDMETHODCALLTYPE statusCode( 
        /* [retval][out] */ int *statusCode);

    virtual HRESULT STDMETHODCALLTYPE isAttachment( 
        /* [retval][out] */ BOOL *attachment);

    // IWebURLResponsePrivate
    virtual HRESULT STDMETHODCALLTYPE sslPeerCertificate( 
        /* [retval][out] */ OLE_HANDLE *result);
    
    const WebCore::ResourceResponse& resourceResponse() const;

protected:
    HRESULT suggestedFileExtension(BSTR* result);

#if USE(CFNETWORK)
    CFDictionaryRef certificateDictionary() const;
#endif

protected:
    ULONG m_refCount;
    WebCore::ResourceResponse m_response;

#if USE(CFNETWORK)
    mutable RetainPtr<CFDictionaryRef> m_SSLCertificateInfo;    // this ensures certificate contexts are valid for the lifetime of this WebURLResponse.
#endif
};

#endif
