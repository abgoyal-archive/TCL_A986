

#ifndef WebMutableURLRequest_H
#define WebMutableURLRequest_H

#include "WebKit.h"
#pragma warning(push, 0)
#include <WebCore/ResourceRequest.h>
#pragma warning(pop)

namespace WebCore
{
    class FormData;
}

inline WebCore::ResourceRequestCachePolicy core(WebURLRequestCachePolicy policy)
{
    return static_cast<WebCore::ResourceRequestCachePolicy>(policy);
}

inline WebURLRequestCachePolicy kit(WebCore::ResourceRequestCachePolicy policy)
{
    return static_cast<WebURLRequestCachePolicy>(policy);
}

class WebMutableURLRequest : public IWebMutableURLRequest, IWebMutableURLRequestPrivate
{
public:
    static WebMutableURLRequest* createInstance();
    static WebMutableURLRequest* createInstance(IWebMutableURLRequest* req);
    static WebMutableURLRequest* createInstance(const WebCore::ResourceRequest&);

    static WebMutableURLRequest* createImmutableInstance();
    static WebMutableURLRequest* createImmutableInstance(const WebCore::ResourceRequest&);
protected:
    WebMutableURLRequest(bool isMutable);
    ~WebMutableURLRequest();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebURLRequest
    virtual HRESULT STDMETHODCALLTYPE requestWithURL( 
        /* [in] */ BSTR theURL,
        /* [optional][in] */ WebURLRequestCachePolicy cachePolicy,
        /* [optional][in] */ double timeoutInterval);
    
    virtual HRESULT STDMETHODCALLTYPE allHTTPHeaderFields( 
        /* [retval][out] */ IPropertyBag **result);
    
    virtual HRESULT STDMETHODCALLTYPE cachePolicy( 
        /* [retval][out] */ WebURLRequestCachePolicy *result);
    
    virtual HRESULT STDMETHODCALLTYPE HTTPBody( 
        /* [retval][out] */ IStream **result);
    
    virtual HRESULT STDMETHODCALLTYPE HTTPBodyStream( 
        /* [retval][out] */ IStream **result);
    
    virtual HRESULT STDMETHODCALLTYPE HTTPMethod( 
        /* [retval][out] */ BSTR *result);
    
    virtual HRESULT STDMETHODCALLTYPE HTTPShouldHandleCookies( 
        /* [retval][out] */ BOOL *result);
    
    virtual HRESULT STDMETHODCALLTYPE initWithURL( 
        /* [in] */ BSTR url,
        /* [optional][in] */ WebURLRequestCachePolicy cachePolicy,
        /* [optional][in] */ double timeoutInterval);
    
    virtual HRESULT STDMETHODCALLTYPE mainDocumentURL( 
        /* [retval][out] */ BSTR *result);
    
    virtual HRESULT STDMETHODCALLTYPE timeoutInterval( 
        /* [retval][out] */ double *result);
    
    virtual HRESULT STDMETHODCALLTYPE URL( 
        /* [retval][out] */ BSTR *result);
    
    virtual HRESULT STDMETHODCALLTYPE valueForHTTPHeaderField( 
        /* [in] */ BSTR field,
        /* [retval][out] */ BSTR *result);

    virtual HRESULT STDMETHODCALLTYPE isEmpty(
    /* [retval][out] */ BOOL* result);

    virtual HRESULT STDMETHODCALLTYPE mutableCopy(
        /* [out, retval] */ IWebMutableURLRequest** result);

    virtual HRESULT STDMETHODCALLTYPE isEqual(
        /* [in] */ IWebURLRequest* other,
        /* [out, retval] */ BOOL* result);

    // IWebMutableURLRequest
    virtual HRESULT STDMETHODCALLTYPE addValue( 
        /* [in] */ BSTR value,
        /* [in] */ BSTR field);
    
    virtual HRESULT STDMETHODCALLTYPE setAllHTTPHeaderFields( 
        /* [in] */ IPropertyBag *headerFields);
    
    virtual HRESULT STDMETHODCALLTYPE setCachePolicy( 
        /* [in] */ WebURLRequestCachePolicy policy);
    
    virtual HRESULT STDMETHODCALLTYPE setHTTPBody( 
        /* [in] */ IStream *data);
    
    virtual HRESULT STDMETHODCALLTYPE setHTTPBodyStream( 
        /* [in] */ IStream *data);
    
    virtual HRESULT STDMETHODCALLTYPE setHTTPMethod( 
        /* [in] */ BSTR method);
    
    virtual HRESULT STDMETHODCALLTYPE setHTTPShouldHandleCookies( 
        /* [in] */ BOOL handleCookies);
    
    virtual HRESULT STDMETHODCALLTYPE setMainDocumentURL( 
        /* [in] */ BSTR theURL);
    
    virtual HRESULT STDMETHODCALLTYPE setTimeoutInterval( 
        /* [in] */ double timeoutInterval);
    
    virtual HRESULT STDMETHODCALLTYPE setURL( 
        /* [in] */ BSTR theURL);
    
    virtual HRESULT STDMETHODCALLTYPE setValue( 
        /* [in] */ BSTR value,
        /* [in] */ BSTR field);

    virtual HRESULT STDMETHODCALLTYPE setAllowsAnyHTTPSCertificate(void);

    // IWebMutableURLRequestPrivate

    virtual HRESULT STDMETHODCALLTYPE setClientCertificate(
        /* [in] */ OLE_HANDLE cert);

    virtual /* [local] */ CFURLRequestRef STDMETHODCALLTYPE cfRequest();

    // WebMutableURLRequest
    void setFormData(const PassRefPtr<WebCore::FormData> data);
    const PassRefPtr<WebCore::FormData> formData() const;
    
    void addHTTPHeaderFields(const WebCore::HTTPHeaderMap& headerFields);
    const WebCore::HTTPHeaderMap& httpHeaderFields() const;

    const WebCore::ResourceRequest& resourceRequest() const;
protected:
    ULONG m_refCount;
    bool m_isMutable;
    WebCore::ResourceRequest m_request;
};

#endif
