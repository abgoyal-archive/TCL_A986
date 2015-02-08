

#ifndef WebDataSource_H
#define WebDataSource_H

#include "WebKit.h"
#include "COMPtr.h"
#include <WTF/RefPtr.h>

class WebDocumentLoader;
class WebMutableURLRequest;

extern const GUID IID_WebDataSource;

class WebDataSource : public IWebDataSource, public IWebDataSourcePrivate
{
public:
    static WebDataSource* createInstance(WebDocumentLoader*);
protected:
    WebDataSource(WebDocumentLoader*);
    ~WebDataSource();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebDataSource
    virtual HRESULT STDMETHODCALLTYPE initWithRequest( 
        /* [in] */ IWebURLRequest *request);
    
    virtual HRESULT STDMETHODCALLTYPE data( 
        /* [retval][out] */ IStream **stream);
    
    virtual HRESULT STDMETHODCALLTYPE representation( 
        /* [retval][out] */ IWebDocumentRepresentation **rep);
    
    virtual HRESULT STDMETHODCALLTYPE webFrame( 
        /* [retval][out] */ IWebFrame **frame);
    
    virtual HRESULT STDMETHODCALLTYPE initialRequest( 
        /* [retval][out] */ IWebURLRequest **request);
    
    virtual HRESULT STDMETHODCALLTYPE request( 
        /* [retval][out] */ IWebMutableURLRequest **request);
    
    virtual HRESULT STDMETHODCALLTYPE response( 
        /* [retval][out] */ IWebURLResponse **response);
    
    virtual HRESULT STDMETHODCALLTYPE textEncodingName( 
        /* [retval][out] */ BSTR *name);
    
    virtual HRESULT STDMETHODCALLTYPE isLoading( 
        /* [retval][out] */ BOOL *loading);
    
    virtual HRESULT STDMETHODCALLTYPE pageTitle( 
        /* [retval][out] */ BSTR *title);
    
    virtual HRESULT STDMETHODCALLTYPE unreachableURL( 
        /* [retval][out] */ BSTR *url);
    
    virtual HRESULT STDMETHODCALLTYPE webArchive( 
        /* [retval][out] */ IWebArchive **archive);
    
    virtual HRESULT STDMETHODCALLTYPE mainResource( 
        /* [retval][out] */ IWebResource **resource);
    
    virtual HRESULT STDMETHODCALLTYPE subresources( 
        /* [retval][out] */ IEnumVARIANT **enumResources);
    
    virtual HRESULT STDMETHODCALLTYPE subresourceForURL( 
        /* [in] */ BSTR url,
        /* [retval][out] */ IWebResource **resource);
    
    virtual HRESULT STDMETHODCALLTYPE addSubresource( 
        /* [in] */ IWebResource *subresource);

    // IWebDataSourcePrivate

    virtual HRESULT STDMETHODCALLTYPE overrideEncoding( 
        /* [retval][out] */ BSTR *encoding);
    
    virtual HRESULT STDMETHODCALLTYPE setOverrideEncoding( 
        /* [in] */ BSTR encoding);

    virtual HRESULT STDMETHODCALLTYPE mainDocumentError(
        /* [retval][out] */ IWebError** error);

    // WebDataSource
    WebDocumentLoader* documentLoader() const;
protected:
    ULONG m_refCount;
    RefPtr<WebDocumentLoader> m_loader;
    COMPtr<IWebDocumentRepresentation> m_representation;
};

#endif
