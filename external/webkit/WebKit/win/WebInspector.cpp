

#include "config.h"
#include "WebInspector.h"

#include "WebKitDLL.h"
#include "WebView.h"
#pragma warning(push, 0)
#include <WebCore/InspectorController.h>
#include <WebCore/Page.h>
#pragma warning(pop)
#include <wtf/Assertions.h>

using namespace WebCore;

WebInspector* WebInspector::createInstance(WebView* webView)
{
    WebInspector* inspector = new WebInspector(webView);
    inspector->AddRef();
    return inspector;
}

WebInspector::WebInspector(WebView* webView)
    : m_refCount(0)
    , m_webView(webView)
{
    ASSERT_ARG(webView, webView);

    gClassCount++;
    gClassNameCount.add("WebInspector");
}

WebInspector::~WebInspector()
{
    gClassCount--;
    gClassNameCount.remove("WebInspector");
}

void WebInspector::webViewClosed()
{
    m_webView = 0;
}

HRESULT STDMETHODCALLTYPE WebInspector::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IWebInspector))
        *ppvObject = static_cast<IWebInspector*>(this);
    else if (IsEqualGUID(riid, IID_IWebInspectorPrivate))
        *ppvObject = static_cast<IWebInspectorPrivate*>(this);
    else if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<IWebInspector*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebInspector::AddRef(void)
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebInspector::Release(void)
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete this;

    return newRef;
}

HRESULT STDMETHODCALLTYPE WebInspector::show()
{
    if (m_webView)
        if (Page* page = m_webView->page())
            page->inspectorController()->show();

    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebInspector::showConsole()
{
    if (m_webView)
        if (Page* page = m_webView->page())
            page->inspectorController()->showPanel(InspectorController::ConsolePanel);

    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebInspector::unused1()
{
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebInspector::close()
{
    if (m_webView)
        if (Page* page = m_webView->page())
            page->inspectorController()->close();

    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebInspector::attach()
{
    if (m_webView)
        if (Page* page = m_webView->page())
            page->inspectorController()->attachWindow();

    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebInspector::detach()
{
    if (m_webView)
        if (Page* page = m_webView->page())
            page->inspectorController()->detachWindow();

    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebInspector::isDebuggingJavaScript(BOOL* isDebugging)
{
    if (!isDebugging)
        return E_POINTER;

    *isDebugging = FALSE;

    if (!m_webView)
        return S_OK;

    Page* page = m_webView->page();
    if (!page)
        return S_OK;

    *isDebugging = page->inspectorController()->debuggerEnabled();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebInspector::toggleDebuggingJavaScript()
{
    if (!m_webView)
        return S_OK;

    Page* page = m_webView->page();
    if (!page)
        return S_OK;

    InspectorController* inspector = page->inspectorController();

    if (inspector->debuggerEnabled())
        inspector->disableDebugger();
    else {
        inspector->showPanel(InspectorController::ScriptsPanel);
        inspector->enableDebugger();
    }

    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebInspector::isProfilingJavaScript(BOOL* isProfiling)
{
    if (!isProfiling)
        return E_POINTER;

    *isProfiling = FALSE;

    if (!m_webView)
        return S_OK;

    Page* page = m_webView->page();
    if (!page)
        return S_OK;

    *isProfiling = page->inspectorController()->isRecordingUserInitiatedProfile();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebInspector::toggleProfilingJavaScript()
{
    if (!m_webView)
        return S_OK;

    Page* page = m_webView->page();
    if (!page)
        return S_OK;

    InspectorController* inspector = page->inspectorController();

    if (inspector->isRecordingUserInitiatedProfile()) {
        inspector->stopUserInitiatedProfiling();
        inspector->showPanel(InspectorController::ProfilesPanel);
    } else
        inspector->startUserInitiatedProfiling();

    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebInspector::isJavaScriptProfilingEnabled(BOOL* isProfilingEnabled)
{
    if (!isProfilingEnabled)
        return E_POINTER;

    *isProfilingEnabled = FALSE;

    if (!m_webView)
        return S_OK;

    Page* page = m_webView->page();
    if (!page)
        return S_OK;

    *isProfilingEnabled = page->inspectorController()->profilerEnabled();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebInspector::setJavaScriptProfilingEnabled(BOOL enabled)
{
    if (!m_webView)
        return S_OK;

    Page* page = m_webView->page();
    if (!page)
        return S_OK;

    if (enabled)
        page->inspectorController()->enableProfiler();
    else
        page->inspectorController()->disableProfiler();

    return S_OK;
}

HRESULT STDMETHODCALLTYPE  WebInspector::evaluateInFrontend(ULONG callId, BSTR bScript)
{
    if (!m_webView)
        return S_OK;

    Page* page = m_webView->page();
    if (!page)
        return S_OK;

    String script(bScript, SysStringLen(bScript));
    page->inspectorController()->evaluateForTestInFrontend(callId, script);
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebInspector::isTimelineProfilingEnabled(BOOL* isEnabled)
{
    if (!isEnabled)
        return E_POINTER;

    *isEnabled = FALSE;

    if (!m_webView)
        return S_OK;

    Page* page = m_webView->page();
    if (!page)
        return S_OK;

    *isEnabled = page->inspectorController()->timelineAgent() != 0;
    return S_OK;
}

HRESULT STDMETHODCALLTYPE WebInspector::setTimelineProfilingEnabled(BOOL enabled)
{
    if (!m_webView)
        return S_OK;

    Page* page = m_webView->page();
    if (!page)
        return S_OK;

    if (enabled)
        page->inspectorController()->startTimelineProfiler();
    else
        page->inspectorController()->stopTimelineProfiler();

    return S_OK;
}
