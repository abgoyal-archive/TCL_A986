

#include "config.h"
#include "WebKitDLL.h"
#include "WebCache.h"

#include "CFDictionaryPropertyBag.h"

#pragma warning(push, 0)
#include <WebCore/ApplicationCacheStorage.h>
#include <WebCore/Cache.h>
#include <WebCore/CrossOriginPreflightResultCache.h>
#pragma warning(pop)

// WebCache ---------------------------------------------------------------------------

WebCache::WebCache()
: m_refCount(0)
{
    gClassCount++;
    gClassNameCount.add("WebCache");
}

WebCache::~WebCache()
{
    gClassCount--;
    gClassNameCount.remove("WebCache");
}

WebCache* WebCache::createInstance()
{
    WebCache* instance = new WebCache();
    instance->AddRef();
    return instance;
}

// IUnknown -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebCache::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<WebCache*>(this);
    else if (IsEqualGUID(riid, IID_IWebCache))
        *ppvObject = static_cast<WebCache*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebCache::AddRef(void)
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebCache::Release(void)
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete(this);

    return newRef;
}

// IWebCache ------------------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebCache::statistics( 
    /* [in][out] */ int* count,
    /* [retval][out] */ IPropertyBag ** s)
{
    if (!count || (s && *count < 4))
        return E_FAIL;

    *count = 4;
    if (!s)
        return S_OK;

    WebCore::Cache::Statistics stat = WebCore::cache()->getStatistics();

    static CFStringRef imagesKey = CFSTR("images");
    static CFStringRef stylesheetsKey = CFSTR("style sheets");
    static CFStringRef xslKey = CFSTR("xsl");
    static CFStringRef scriptsKey = CFSTR("scripts");
#if !ENABLE(XSLT)
    const int zero = 0;
#endif

    RetainPtr<CFMutableDictionaryRef> dictionary(AdoptCF,
        CFDictionaryCreateMutable(0, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks));

    RetainPtr<CFNumberRef> value(AdoptCF, CFNumberCreate(0, kCFNumberIntType, &stat.images.count));
    CFDictionaryAddValue(dictionary.get(), imagesKey, value.get());
    
    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &stat.cssStyleSheets.count));
    CFDictionaryAddValue(dictionary.get(), stylesheetsKey, value.get());
    
#if ENABLE(XSLT)
    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &stat.xslStyleSheets.count));
#else
    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &zero));
#endif
    CFDictionaryAddValue(dictionary.get(), xslKey, value.get());

    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &stat.scripts.count));
    CFDictionaryAddValue(dictionary.get(), scriptsKey, value.get());

    COMPtr<CFDictionaryPropertyBag> propBag = CFDictionaryPropertyBag::createInstance();
    propBag->setDictionary(dictionary.get());
    s[0] = propBag.releaseRef();

    dictionary.adoptCF(CFDictionaryCreateMutable(0, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks));

    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &stat.images.size));
    CFDictionaryAddValue(dictionary.get(), imagesKey, value.get());
    
    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &stat.cssStyleSheets.size));
    CFDictionaryAddValue(dictionary.get(), stylesheetsKey, value.get());
    
#if ENABLE(XSLT)
    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &stat.xslStyleSheets.size));
#else
    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &zero));
#endif
    CFDictionaryAddValue(dictionary.get(), xslKey, value.get());

    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &stat.scripts.size));
    CFDictionaryAddValue(dictionary.get(), scriptsKey, value.get());

    propBag = CFDictionaryPropertyBag::createInstance();
    propBag->setDictionary(dictionary.get());
    s[1] = propBag.releaseRef();

    dictionary.adoptCF(CFDictionaryCreateMutable(0, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks));

    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &stat.images.liveSize));
    CFDictionaryAddValue(dictionary.get(), imagesKey, value.get());
    
    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &stat.cssStyleSheets.liveSize));
    CFDictionaryAddValue(dictionary.get(), stylesheetsKey, value.get());
    
#if ENABLE(XSLT)
    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &stat.xslStyleSheets.liveSize));
#else
    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &zero));
#endif
    CFDictionaryAddValue(dictionary.get(), xslKey, value.get());

    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &stat.scripts.liveSize));
    CFDictionaryAddValue(dictionary.get(), scriptsKey, value.get());

    propBag = CFDictionaryPropertyBag::createInstance();
    propBag->setDictionary(dictionary.get());
    s[2] = propBag.releaseRef();

    dictionary.adoptCF(CFDictionaryCreateMutable(0, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks));

    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &stat.images.decodedSize));
    CFDictionaryAddValue(dictionary.get(), imagesKey, value.get());
    
    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &stat.cssStyleSheets.decodedSize));
    CFDictionaryAddValue(dictionary.get(), stylesheetsKey, value.get());
    
#if ENABLE(XSLT)
    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &stat.xslStyleSheets.decodedSize));
#else
    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &zero));
#endif
    CFDictionaryAddValue(dictionary.get(), xslKey, value.get());

    value.adoptCF(CFNumberCreate(0, kCFNumberIntType, &stat.scripts.decodedSize));
    CFDictionaryAddValue(dictionary.get(), scriptsKey, value.get());

    propBag = CFDictionaryPropertyBag::createInstance();
    propBag->setDictionary(dictionary.get());
    s[3] = propBag.releaseRef();

    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebCache::empty( void)
{
    if (WebCore::cache()->disabled())
        return S_OK;
    WebCore::cache()->setDisabled(true);
    WebCore::cache()->setDisabled(false);

    // Empty the application cache.
    WebCore::cacheStorage().empty();

    // Empty the Cross-Origin Preflight cache
    WebCore::CrossOriginPreflightResultCache::shared().empty();

    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebCache::setDisabled( 
    /* [in] */ BOOL disabled)
{
    WebCore::cache()->setDisabled(!!disabled);
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebCache::disabled(
    /* [out][retval] */ BOOL* disabled)
{
    if (!disabled)
        return E_POINTER;
    *disabled = WebCore::cache()->disabled();
    return S_OK;
}
