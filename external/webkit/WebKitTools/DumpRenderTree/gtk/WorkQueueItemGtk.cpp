

#include "config.h"
#include "WorkQueueItem.h"

#include "DumpRenderTree.h"

#include <JavaScriptCore/JSStringRef.h>
#include <webkit/webkit.h>
#include <string.h>

// Returns a newly allocated UTF-8 character buffer which must be freed with g_free()
gchar* JSStringCopyUTF8CString(JSStringRef jsString)
{
    size_t dataSize = JSStringGetMaximumUTF8CStringSize(jsString);
    gchar* utf8 = (gchar*)g_malloc(dataSize);
    JSStringGetUTF8CString(jsString, utf8, dataSize);

    return utf8;
}

bool LoadItem::invoke() const
{
    gchar* targetString = JSStringCopyUTF8CString(m_target.get());

    WebKitWebFrame* targetFrame;
    if (!strlen(targetString))
        targetFrame = mainFrame;
    else
        targetFrame = webkit_web_frame_find_frame(mainFrame, targetString);
    g_free(targetString);

    gchar* urlString = JSStringCopyUTF8CString(m_url.get());
    WebKitNetworkRequest* request = webkit_network_request_new(urlString);
    g_free(urlString);
    webkit_web_frame_load_request(targetFrame, request);
    g_object_unref(request);

    return true;
}

bool ReloadItem::invoke() const
{
    webkit_web_frame_reload(mainFrame);
    return true;
}

bool ScriptItem::invoke() const
{
    WebKitWebView* webView = webkit_web_frame_get_web_view(mainFrame);
    gchar* scriptString = JSStringCopyUTF8CString(m_script.get());
    webkit_web_view_execute_script(webView, scriptString);
    g_free(scriptString);
    return true;
}

bool BackForwardItem::invoke() const
{
    WebKitWebView* webView = webkit_web_frame_get_web_view(mainFrame);
    if (m_howFar == 1)
        webkit_web_view_go_forward(webView);
    else if (m_howFar == -1)
        webkit_web_view_go_back(webView);
    else {
        WebKitWebBackForwardList* webBackForwardList = webkit_web_view_get_back_forward_list(webView);
        WebKitWebHistoryItem* item = webkit_web_back_forward_list_get_nth_item(webBackForwardList, m_howFar);
        webkit_web_view_go_to_back_forward_item(webView, item);
    }
    return true;
}
