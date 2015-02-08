

#include "config.h"
#include "WebKitDLL.h"
#include "WebResource.h"

#include "MarshallingHelpers.h"
#include "MemoryStream.h"

#pragma warning(push, 0)
#include <WebCore/BString.h>
#pragma warning(pop)

using namespace WebCore;

// WebResource ---------------------------------------------------------------------

WebResource::WebResource(IStream* data, const WebCore::KURL& url, const WebCore::String& mimeType, const WebCore::String& textEncodingName, const WebCore::String& frameName)
    : m_refCount(0)
    , m_data(data)
    , m_url(url)
    , m_mimeType(mimeType)
    , m_textEncodingName(textEncodingName)
    , m_frameName(frameName)
{
    gClassCount++;
    gClassNameCount.add("WebResource");
}

WebResource::~WebResource()
{
    gClassCount--;
    gClassNameCount.remove("WebResource");
}

WebResource* WebResource::createInstance(PassRefPtr<WebCore::SharedBuffer> data, const WebCore::ResourceResponse& response)
{
    COMPtr<MemoryStream> memoryStream = MemoryStream::createInstance(data);

    WebResource* instance = new WebResource(memoryStream.get(), response.url(), response.mimeType(), response.textEncodingName(), String());
    instance->AddRef();
    return instance;
}

// IUnknown -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebResource::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<IUnknown*>(this);
    else if (IsEqualGUID(riid, IID_IWebResource))
        *ppvObject = static_cast<IWebResource*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebResource::AddRef(void)
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebResource::Release(void)
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete(this);

    return newRef;
}

// WebResource ------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebResource::initWithData( 
    /* [in] */ IStream *data,
    /* [in] */ BSTR url,
    /* [in] */ BSTR mimeType,
    /* [in] */ BSTR textEncodingName,
    /* [in] */ BSTR frameName)
{
    m_data = data;
    m_url = MarshallingHelpers::BSTRToKURL(url);
    m_mimeType = String(mimeType);
    m_textEncodingName = String(textEncodingName);
    m_frameName = String(frameName);

    return S_OK;
}

    
HRESULT STDMETHODCALLTYPE WebResource::data( 
    /* [retval][out] */ IStream **data)
{
    return m_data.copyRefTo(data);
}
   
HRESULT STDMETHODCALLTYPE WebResource::URL( 
    /* [retval][out] */ BSTR *url)
{
    if (!url) {
        ASSERT_NOT_REACHED();
        return E_POINTER;
    }

    *url = BString(String(m_url.string())).release();
    return S_OK;
}
    
HRESULT STDMETHODCALLTYPE WebResource::MIMEType( 
    /* [retval][out] */ BSTR *mime)
{
    if (!mime) {
        ASSERT_NOT_REACHED();
        return E_POINTER;
    }

    *mime = BString(m_mimeType).release();
    return S_OK;
}
   
HRESULT STDMETHODCALLTYPE WebResource::textEncodingName( 
    /* [retval][out] */ BSTR *encodingName)
{
    if (!encodingName) {
        ASSERT_NOT_REACHED();
        return E_POINTER;
    }

    *encodingName = BString(m_textEncodingName).release();
    return S_OK;
}
    
HRESULT STDMETHODCALLTYPE WebResource::frameName( 
    /* [retval][out] */ BSTR *name)
{
    if (!name) {
        ASSERT_NOT_REACHED();
        return E_POINTER;
    }

    *name = BString(m_frameName).release();
    return S_OK;
}
