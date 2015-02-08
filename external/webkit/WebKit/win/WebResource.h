

#ifndef WebResource_h
#define WebResource_h

#include "WebKit.h"
#include "COMPtr.h"

#pragma warning(push, 0)
#include <WebCore/KURL.h>
#include <WebCore/PlatformString.h>
#include <WebCore/ResourceResponse.h>
#include <WebCore/SharedBuffer.h>
#include <wtf/PassRefPtr.h>
#pragma warning(pop)

class WebResource : public IWebResource {
public:
    static WebResource* createInstance(PassRefPtr<WebCore::SharedBuffer> data, const WebCore::ResourceResponse& response);
protected:
    WebResource(IStream* data, const WebCore::KURL& url, const WebCore::String& mimeType, const WebCore::String& textEncodingName, const WebCore::String& frameName);
    ~WebResource();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebResource
    virtual HRESULT STDMETHODCALLTYPE initWithData( 
        /* [in] */ IStream *data,
        /* [in] */ BSTR url,
        /* [in] */ BSTR mimeType,
        /* [in] */ BSTR textEncodingName,
        /* [in] */ BSTR frameName);
        
    virtual HRESULT STDMETHODCALLTYPE data( 
        /* [retval][out] */ IStream **data);
       
    virtual HRESULT STDMETHODCALLTYPE URL( 
        /* [retval][out] */ BSTR *url);
        
    virtual HRESULT STDMETHODCALLTYPE MIMEType( 
        /* [retval][out] */ BSTR *mime);
        
    virtual HRESULT STDMETHODCALLTYPE textEncodingName( 
        /* [retval][out] */ BSTR *encodingName);
        
    virtual HRESULT STDMETHODCALLTYPE frameName( 
        /* [retval][out] */ BSTR *name);

private:
    ULONG m_refCount;
    COMPtr<IStream> m_data;
    WebCore::KURL m_url;
    WebCore::String m_mimeType;
    WebCore::String m_textEncodingName;
    WebCore::String m_frameName;
};

#endif
