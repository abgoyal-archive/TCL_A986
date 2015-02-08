

#include "config.h"
#include "WebKitDLL.h"
#include "WebArchive.h"

#include "DOMCoreClasses.h"
#include "MemoryStream.h"
#include <WebCore/LegacyWebArchive.h>

using namespace WebCore;

// WebArchive ----------------------------------------------------------------

WebArchive* WebArchive::createInstance()
{
    WebArchive* instance = new WebArchive(0);
    instance->AddRef();
    return instance;
}

WebArchive* WebArchive::createInstance(PassRefPtr<LegacyWebArchive> coreArchive)
{
    WebArchive* instance = new WebArchive(coreArchive);

    instance->AddRef();
    return instance;
}

WebArchive::WebArchive(PassRefPtr<LegacyWebArchive> coreArchive)
    : m_refCount(0)
    , m_archive(coreArchive)
{
    gClassCount++;
    gClassNameCount.add("WebArchive");
}

WebArchive::~WebArchive()
{
    gClassCount--;
    gClassNameCount.remove("WebArchive");
}

HRESULT STDMETHODCALLTYPE WebArchive::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<IWebArchive*>(this);
    else if (IsEqualGUID(riid, __uuidof(IWebArchive)))
        *ppvObject = static_cast<IWebArchive*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebArchive::AddRef()
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebArchive::Release()
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete(this);

    return newRef;
}

HRESULT STDMETHODCALLTYPE WebArchive::initWithMainResource(
        /* [in] */ IWebResource*, 
        /* [in, size_is(cSubResources)] */ IWebResource**, 
        /* [in] */ int, 
        /* in, size_is(cSubFrameArchives)] */ IWebArchive**, 
        /* [in] */ int)
{
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE WebArchive::initWithData(
        /* [in] */ IStream*)
{
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE WebArchive::initWithNode(
        /* [in] */ IDOMNode* node)
{
    if (!node)
        return E_POINTER;

    COMPtr<DOMNode> domNode(Query, node);
    if (!domNode)
        return E_NOINTERFACE;

    m_archive = LegacyWebArchive::create(domNode->node());
    
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebArchive::mainResource(
        /* [out, retval] */ IWebResource**)
{
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE WebArchive::subResources(
        /* [out, retval] */ IEnumVARIANT**)
{
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE WebArchive::subframeArchives(
        /* [out, retval] */ IEnumVARIANT**)
{
    return E_NOTIMPL;
}

HRESULT STDMETHODCALLTYPE WebArchive::data(
        /* [out, retval] */ IStream** stream)
{
    RetainPtr<CFDataRef> cfData = m_archive->rawDataRepresentation();
    if (!cfData)
        return E_FAIL;

    RefPtr<SharedBuffer> buffer = SharedBuffer::create(CFDataGetBytePtr(cfData.get()), CFDataGetLength(cfData.get()));

    return MemoryStream::createInstance(buffer).copyRefTo(stream);
}
