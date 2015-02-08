

#include "config.h"
#include "WebKitDLL.h"
#include "WebCoreStatistics.h"

#include "COMPropertyBag.h"
#include <JavaScriptCore/JSLock.h>
#include <WebCore/FontCache.h>
#include <WebCore/GlyphPageTreeNode.h>
#include <WebCore/IconDatabase.h>
#include <WebCore/JSDOMWindow.h>

using namespace JSC;
using namespace WebCore;

// WebCoreStatistics ---------------------------------------------------------------------------

WebCoreStatistics::WebCoreStatistics()
: m_refCount(0)
{
    gClassCount++;
    gClassNameCount.add("WebCoreStatistics");
}

WebCoreStatistics::~WebCoreStatistics()
{
    gClassCount--;
    gClassNameCount.remove("WebCoreStatistics");
}

WebCoreStatistics* WebCoreStatistics::createInstance()
{
    WebCoreStatistics* instance = new WebCoreStatistics();
    instance->AddRef();
    return instance;
}

// IUnknown -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebCoreStatistics::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<WebCoreStatistics*>(this);
    else if (IsEqualGUID(riid, IID_IWebCoreStatistics))
        *ppvObject = static_cast<WebCoreStatistics*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebCoreStatistics::AddRef(void)
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebCoreStatistics::Release(void)
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete(this);

    return newRef;
}

// IWebCoreStatistics ------------------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebCoreStatistics::javaScriptObjectsCount( 
    /* [retval][out] */ UINT* count)
{
    if (!count)
        return E_POINTER;

    JSLock lock(SilenceAssertionsOnly);
    *count = (UINT)JSDOMWindow::commonJSGlobalData()->heap.objectCount();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebCoreStatistics::javaScriptGlobalObjectsCount( 
    /* [retval][out] */ UINT* count)
{
    if (!count)
        return E_POINTER;

    JSLock lock(SilenceAssertionsOnly);
    *count = (UINT)JSDOMWindow::commonJSGlobalData()->heap.globalObjectCount();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebCoreStatistics::javaScriptProtectedObjectsCount( 
    /* [retval][out] */ UINT* count)
{
    if (!count)
        return E_POINTER;

    JSLock lock(SilenceAssertionsOnly);
    *count = (UINT)JSDOMWindow::commonJSGlobalData()->heap.protectedObjectCount();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebCoreStatistics::javaScriptProtectedGlobalObjectsCount( 
    /* [retval][out] */ UINT* count)
{
    if (!count)
        return E_POINTER;

    JSLock lock(SilenceAssertionsOnly);
    *count = (UINT)JSDOMWindow::commonJSGlobalData()->heap.protectedGlobalObjectCount();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebCoreStatistics::javaScriptProtectedObjectTypeCounts( 
    /* [retval][out] */ IPropertyBag2** typeNamesAndCounts)
{
    JSLock lock(SilenceAssertionsOnly);
    OwnPtr<HashCountedSet<const char*> > jsObjectTypeNames(JSDOMWindow::commonJSGlobalData()->heap.protectedObjectTypeCounts());
    typedef HashCountedSet<const char*>::const_iterator Iterator;
    Iterator end = jsObjectTypeNames->end();
    HashMap<String, int> typeCountMap;
    for (Iterator current = jsObjectTypeNames->begin(); current != end; ++current)
        typeCountMap.set(current->first, current->second);

    COMPtr<IPropertyBag2> results(AdoptCOM, COMPropertyBag<int>::createInstance(typeCountMap));
    results.copyRefTo(typeNamesAndCounts);
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebCoreStatistics::iconPageURLMappingCount( 
    /* [retval][out] */ UINT* count)
{
    if (!count)
        return E_POINTER;
    *count = (UINT) iconDatabase()->pageURLMappingCount();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebCoreStatistics::iconRetainedPageURLCount( 
    /* [retval][out] */ UINT *count)
{
    if (!count)
        return E_POINTER;
    *count = (UINT) iconDatabase()->retainedPageURLCount();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebCoreStatistics::iconRecordCount( 
    /* [retval][out] */ UINT *count)
{
    if (!count)
        return E_POINTER;
    *count = (UINT) iconDatabase()->iconRecordCount();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebCoreStatistics::iconsWithDataCount( 
    /* [retval][out] */ UINT *count)
{
    if (!count)
        return E_POINTER;
    *count = (UINT) iconDatabase()->iconRecordCountWithData();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebCoreStatistics::cachedFontDataCount( 
    /* [retval][out] */ UINT *count)
{
    if (!count)
        return E_POINTER;
    *count = (UINT) fontCache()->fontDataCount();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebCoreStatistics::cachedFontDataInactiveCount( 
    /* [retval][out] */ UINT *count)
{
    if (!count)
        return E_POINTER;
    *count = (UINT) fontCache()->inactiveFontDataCount();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebCoreStatistics::purgeInactiveFontData(void)
{
    fontCache()->purgeInactiveFontData();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebCoreStatistics::glyphPageCount( 
    /* [retval][out] */ UINT *count)
{
    if (!count)
        return E_POINTER;
    *count = (UINT) GlyphPageTreeNode::treeGlyphPageCount();
    return S_OK;
}
