

#include "config.h"
#include "WebKitDLL.h"
#include "WebFramePolicyListener.h"

#include "WebFrame.h"

#pragma warning(push, 0)
#include <WebCore/Frame.h>
#include <WebCore/FrameLoader.h>
#include <WebCore/FrameLoaderClient.h>
#pragma warning(pop)

using namespace WebCore;

// WebFramePolicyListener ----------------------------------------------------------------

WebFramePolicyListener::WebFramePolicyListener(PassRefPtr<Frame> frame)
    : m_refCount(0)
    , m_frame(frame)
{
    gClassCount++;
    gClassNameCount.add("WebFramePolicyListener");
}

WebFramePolicyListener::~WebFramePolicyListener()
{
    gClassCount--;
    gClassNameCount.remove("WebFramePolicyListener");
}

WebFramePolicyListener* WebFramePolicyListener::createInstance(PassRefPtr<Frame> frame)
{
    WebFramePolicyListener* instance = new WebFramePolicyListener(frame);
    instance->AddRef();
    return instance;
}

// IUnknown -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebFramePolicyListener::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<IWebPolicyDecisionListener*>(this);
    else if (IsEqualGUID(riid, IID_IWebPolicyDecisionListener))
        *ppvObject = static_cast<IWebPolicyDecisionListener*>(this);
    else if (IsEqualGUID(riid, IID_IWebFormSubmissionListener))
        *ppvObject = static_cast<IWebFormSubmissionListener*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebFramePolicyListener::AddRef(void)
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebFramePolicyListener::Release(void)
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete(this);

    return newRef;
}

// IWebPolicyDecisionListener ------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebFramePolicyListener::use(void)
{
    receivedPolicyDecision(PolicyUse);
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebFramePolicyListener::download(void)
{
    receivedPolicyDecision(PolicyDownload);
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebFramePolicyListener::ignore(void)
{
    receivedPolicyDecision(PolicyIgnore);
    return S_OK;
}

// IWebFormSubmissionListener ------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebFramePolicyListener::continueSubmit(void)
{
    receivedPolicyDecision(PolicyUse);
    return S_OK;
}

// WebFramePolicyListener ----------------------------------------------------------------
void WebFramePolicyListener::receivedPolicyDecision(PolicyAction action)
{
    RefPtr<Frame> frame = m_frame.release();
    if (frame)
        static_cast<WebFrame*>(frame->loader()->client())->receivedPolicyDecision(action);
}

void WebFramePolicyListener::invalidate()
{
    m_frame = 0;
}

