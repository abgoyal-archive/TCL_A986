

#include "config.h"
#include "WebTextRenderer.h"

#include "WebKitDLL.h"

#include <CoreFoundation/CFString.h>
#if PLATFORM(CG)
#include <WebKitSystemInterface/WebKitSystemInterface.h>
#endif
#include <wtf/RetainPtr.h>

WebTextRenderer* WebTextRenderer::createInstance()
{
    WebTextRenderer* instance = new WebTextRenderer;
    instance->AddRef();
    return instance;
}

WebTextRenderer::WebTextRenderer()
    : m_refCount(0)
{
    gClassCount++;
    gClassNameCount.add("WebTextRenderer");
}

WebTextRenderer::~WebTextRenderer()
{
    gClassCount--;
    gClassNameCount.remove("WebTextRenderer");
}

HRESULT STDMETHODCALLTYPE WebTextRenderer::QueryInterface(const IID &riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<IWebTextRenderer*>(this);
    else if (IsEqualGUID(riid, IID_IWebTextRenderer))
        *ppvObject = static_cast<IWebTextRenderer*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebTextRenderer::AddRef()
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebTextRenderer::Release()
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete this;

    return newRef;
}

HRESULT STDMETHODCALLTYPE WebTextRenderer::registerPrivateFont(
    /* [in] */ LPCOLESTR fontFilePath)
{
    if (!AddFontResourceEx(fontFilePath, FR_PRIVATE, 0))
        return E_FAIL;

    RetainPtr<CFStringRef> string(AdoptCF, CFStringCreateWithCharacters(0, reinterpret_cast<const UniChar*>(fontFilePath), static_cast<CFIndex>(wcslen(fontFilePath))));
#if PLATFORM(CG)
    wkAddFontsAtPath(string.get());
#endif
    return S_OK;
}
