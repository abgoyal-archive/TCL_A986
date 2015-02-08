

#include "config.h"
#include "WebInspectorDelegate.h"

WebInspectorDelegate::WebInspectorDelegate()
    :m_refCount(0)
{
}

WebInspectorDelegate* WebInspectorDelegate::createInstance()
{
    WebInspectorDelegate* instance = new WebInspectorDelegate;
    instance->AddRef();
    return instance;
}

ULONG STDMETHODCALLTYPE WebInspectorDelegate::AddRef()
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebInspectorDelegate::Release()
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete(this);

    return newRef;
}

HRESULT STDMETHODCALLTYPE WebInspectorDelegate::dragDestinationActionMaskForDraggingInfo(
    /* [in] */ IWebView*,
    /* [in] */ IDataObject*,
    /* [retval][out] */ WebDragDestinationAction* action)
{
    *action = WebDragDestinationActionNone;

    return S_OK;
}
