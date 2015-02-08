

#include "config.h"
#include "WorkQueueItem.h"

#include "DumpRenderTree.h"
#include <WebCore/COMPtr.h>
#include <WebKit/WebKit.h>
#include <WebKit/WebKitCOMAPI.h>
#include <JavaScriptCore/JSStringRef.h>
#include <JavaScriptCore/JSStringRefCF.h>
#include <JavaScriptCore/RetainPtr.h>
#include <wtf/Vector.h>
#include <string>

using std::wstring;

static wstring jsStringRefToWString(JSStringRef jsStr)
{
    size_t length = JSStringGetLength(jsStr);
    Vector<WCHAR> buffer(length + 1);
    memcpy(buffer.data(), JSStringGetCharactersPtr(jsStr), length * sizeof(WCHAR));
    buffer[length] = '\0';

    return buffer.data();
}

bool LoadItem::invoke() const
{
    wstring targetString = jsStringRefToWString(m_target.get());

    COMPtr<IWebFrame> targetFrame;
    if (targetString.empty())
        targetFrame = frame;
    else {
        BSTR targetBSTR = SysAllocString(targetString.c_str());
        bool failed = FAILED(frame->findFrameNamed(targetBSTR, &targetFrame));
        SysFreeString(targetBSTR);
        if (failed)
            return false;
    }

    COMPtr<IWebURLRequest> request;
    if (FAILED(WebKitCreateInstance(CLSID_WebURLRequest, 0, IID_IWebURLRequest, (void**)&request)))
        return false;

    wstring urlString = jsStringRefToWString(m_url.get());
    BSTR urlBSTR = SysAllocString(urlString.c_str());
    bool failed = FAILED(request->initWithURL(urlBSTR, WebURLRequestUseProtocolCachePolicy, 60));
    SysFreeString(urlBSTR);
    if (failed)
        return false;

    targetFrame->loadRequest(request.get());
    return true;
}

bool ReloadItem::invoke() const
{
    COMPtr<IWebView> webView;
    if (FAILED(frame->webView(&webView)))
        return false;

    COMPtr<IWebIBActions> webActions;
    if (FAILED(webView->QueryInterface(&webActions)))
        return false;

    webActions->reload(0);
    return true;
}

bool ScriptItem::invoke() const
{
    COMPtr<IWebView> webView;
    if (FAILED(frame->webView(&webView)))
        return false;

    wstring scriptString = jsStringRefToWString(m_script.get());

    BSTR result;
    BSTR scriptBSTR = SysAllocString(scriptString.c_str());
    webView->stringByEvaluatingJavaScriptFromString(scriptBSTR, &result);
    SysFreeString(result);
    SysFreeString(scriptBSTR);

    return true;
}

bool BackForwardItem::invoke() const
{
    COMPtr<IWebView> webView;
    if (FAILED(frame->webView(&webView)))
        return false;

    BOOL result;
    if (m_howFar == 1) {
        webView->goForward(&result);
        return true;
    }

    if (m_howFar == -1) {
        webView->goBack(&result);
        return true;
    }
    
    COMPtr<IWebBackForwardList> bfList;
    if (FAILED(webView->backForwardList(&bfList)))
        return false;

    COMPtr<IWebHistoryItem> item;
    if (FAILED(bfList->itemAtIndex(m_howFar, &item)))
        return false;

    webView->goToBackForwardItem(item.get(), &result);
    return true;
}
