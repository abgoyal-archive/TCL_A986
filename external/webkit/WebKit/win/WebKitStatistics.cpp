

#include "config.h"
#include "WebKitDLL.h"
#include "WebKitStatistics.h"

#include "WebKitStatisticsPrivate.h"
#include <WebCore/BString.h>

using namespace WebCore;

int WebViewCount;
int WebDataSourceCount;
int WebFrameCount;
int WebHTMLRepresentationCount;
int WebFrameViewCount;

// WebKitStatistics ---------------------------------------------------------------------------

WebKitStatistics::WebKitStatistics()
: m_refCount(0)
{
    gClassCount++;
    gClassNameCount.add("WebKitStatistics");
}

WebKitStatistics::~WebKitStatistics()
{
    gClassCount--;
    gClassNameCount.remove("WebKitStatistics");
}

WebKitStatistics* WebKitStatistics::createInstance()
{
    WebKitStatistics* instance = new WebKitStatistics();
    instance->AddRef();
    return instance;
}

// IUnknown -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebKitStatistics::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<WebKitStatistics*>(this);
    else if (IsEqualGUID(riid, IID_IWebKitStatistics))
        *ppvObject = static_cast<WebKitStatistics*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebKitStatistics::AddRef(void)
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebKitStatistics::Release(void)
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete(this);

    return newRef;
}

// IWebKitStatistics ------------------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebKitStatistics::webViewCount( 
    /* [retval][out] */ int *count)
{
    *count = WebViewCount;
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebKitStatistics::frameCount( 
    /* [retval][out] */ int *count)
{
    *count = WebFrameCount;
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebKitStatistics::dataSourceCount( 
    /* [retval][out] */ int *count)
{
    *count = WebDataSourceCount;
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebKitStatistics::viewCount( 
    /* [retval][out] */ int *count)
{
    *count = WebFrameViewCount;
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebKitStatistics::HTMLRepresentationCount( 
    /* [retval][out] */ int *count)
{
    *count = WebHTMLRepresentationCount;
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebKitStatistics::comClassCount( 
    /* [retval][out] */ int *classCount)
{
    *classCount = gClassCount;
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebKitStatistics::comClassNameCounts( 
    /* [retval][out] */ BSTR *output)
{
    typedef HashCountedSet<String>::const_iterator Iterator;
    Iterator end = gClassNameCount.end();
    Vector<UChar> vector;
    for (Iterator current = gClassNameCount.begin(); current != end; ++current) {
        append(vector, String::format("%4u", current->second));
        vector.append('\t');
        append(vector, static_cast<String>(current->first));
        vector.append('\n');
    }
    *output = BString(String::adopt(vector)).release();
    return S_OK;
}
