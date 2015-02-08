

#include "config.h"
#include "WebKitDLL.h"
#include "WebJavaScriptCollector.h"

#pragma warning(push, 0)
#include <JavaScriptCore/JSGlobalData.h>
#include <JavaScriptCore/collector.h>
#include <WebCore/GCController.h>
#include <WebCore/JSDOMWindow.h>
#include <runtime/JSLock.h>
#pragma warning(pop)

using namespace JSC;
using namespace WebCore;

// WebJavaScriptCollector ---------------------------------------------------------------------------

WebJavaScriptCollector::WebJavaScriptCollector()
: m_refCount(0)
{
    gClassCount++;
    gClassNameCount.add("WebJavaScriptCollector");
}

WebJavaScriptCollector::~WebJavaScriptCollector()
{
    gClassCount--;
    gClassNameCount.remove("WebJavaScriptCollector");
}

WebJavaScriptCollector* WebJavaScriptCollector::createInstance()
{
    WebJavaScriptCollector* instance = new WebJavaScriptCollector();
    instance->AddRef();
    return instance;
}

// IUnknown -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebJavaScriptCollector::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<IWebJavaScriptCollector*>(this);
    else if (IsEqualGUID(riid, IID_IWebJavaScriptCollector))
        *ppvObject = static_cast<IWebJavaScriptCollector*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebJavaScriptCollector::AddRef(void)
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebJavaScriptCollector::Release(void)
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete(this);

    return newRef;
}

// IWebJavaScriptCollector ------------------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebJavaScriptCollector::collect()
{
    gcController().garbageCollectNow();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebJavaScriptCollector::collectOnAlternateThread( 
    /* [in] */ BOOL waitUntilDone)
{
    gcController().garbageCollectOnAlternateThreadForDebugging(!!waitUntilDone);
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebJavaScriptCollector::objectCount( 
    /* [retval][out] */ UINT* count)
{
    if (!count) {
        ASSERT_NOT_REACHED();
        return E_POINTER;
    }

    JSLock lock(SilenceAssertionsOnly);
    *count = (UINT)JSDOMWindow::commonJSGlobalData()->heap.objectCount();
    return S_OK;
}
