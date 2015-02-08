

#include "config.h"
#include "WebKitDLL.h"
#include "WebWorkersPrivate.h"

#include <WebCore/WorkerThread.h>

// IUnknown -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebWorkersPrivate::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, __uuidof(IWebWorkersPrivate)))
        *ppvObject = this;
    else if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<IWebWorkersPrivate*>(this);
    else if (IsEqualGUID(riid, IID_IWebWorkersPrivate))
        *ppvObject = static_cast<IWebWorkersPrivate*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebWorkersPrivate::AddRef(void)
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebWorkersPrivate::Release(void)
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete(this);

    return newRef;
}

// IWebWorkersPrivate ---------------------------------------------------------

HRESULT WebWorkersPrivate::workerThreadCount(UINT* number)
{
    if (!number)
        return E_POINTER;

#if ENABLE(WORKERS)
    *number = WebCore::WorkerThread::workerThreadCount();
#else
    *number = 0;
#endif
    return S_OK;
}

// WebWorkersPrivate ----------------------------------------------------------

WebWorkersPrivate* WebWorkersPrivate::createInstance()
{
    WebWorkersPrivate* instance = new WebWorkersPrivate();
    instance->AddRef();
    return instance;
}

WebWorkersPrivate::WebWorkersPrivate()
    : m_refCount(0)
{
    gClassCount++;
    gClassNameCount.add("WebWorkersPrivate");
}

WebWorkersPrivate::~WebWorkersPrivate()
{
    gClassCount--;
    gClassNameCount.remove("WebWorkersPrivate");
}
