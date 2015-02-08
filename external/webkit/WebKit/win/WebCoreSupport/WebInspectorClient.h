

#ifndef WebInspectorClient_h
#define WebInspectorClient_h

#include <WebCore/COMPtr.h>
#include <WebCore/InspectorClient.h>
#include <WebCore/PlatformString.h>
#include <WebCore/WindowMessageListener.h>
#include <wtf/OwnPtr.h>
#include <windows.h>

class WebNodeHighlight;
class WebView;

class WebInspectorClient : public WebCore::InspectorClient, WebCore::WindowMessageListener {
public:
    WebInspectorClient(WebView*);

    // InspectorClient
    virtual void inspectorDestroyed();

    virtual WebCore::Page* createPage();

    virtual WebCore::String localizedStringsURL();

    virtual WebCore::String hiddenPanels();

    virtual void showWindow();
    virtual void closeWindow();
    virtual bool windowVisible();

    virtual void attachWindow();
    virtual void detachWindow();

    virtual void setAttachedWindowHeight(unsigned height);

    virtual void highlight(WebCore::Node*);
    virtual void hideHighlight();

    virtual void inspectedURLChanged(const WebCore::String& newURL);

    virtual void populateSetting(const WebCore::String& key, WebCore::String* value);
    virtual void storeSetting(const WebCore::String& key, const WebCore::String& value);

    virtual void inspectorWindowObjectCleared();

private:
    ~WebInspectorClient();

    void closeWindowWithoutNotifications();
    void showWindowWithoutNotifications();

    void updateWindowTitle();

    LRESULT onGetMinMaxInfo(WPARAM, LPARAM);
    LRESULT onSize(WPARAM, LPARAM);
    LRESULT onClose(WPARAM, LPARAM);
    LRESULT onSetFocus();

    virtual void windowReceivedMessage(HWND, UINT message, WPARAM, LPARAM);

    void onWebViewWindowPosChanging(WPARAM, LPARAM);

    WebView* m_inspectedWebView;
    HWND m_inspectedWebViewHwnd;
    HWND m_hwnd;
    COMPtr<WebView> m_webView;
    HWND m_webViewHwnd;

    bool m_shouldAttachWhenShown;
    bool m_attached;

    OwnPtr<WebNodeHighlight> m_highlight;

    WebCore::String m_inspectedURL;

    static friend LRESULT CALLBACK WebInspectorWndProc(HWND, UINT, WPARAM, LPARAM);
};

#endif // !defined(WebInspectorClient_h)
