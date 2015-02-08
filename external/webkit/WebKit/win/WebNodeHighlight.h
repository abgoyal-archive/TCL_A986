

#ifndef WebNodeHighlight_h
#define WebNodeHighlight_h

#pragma warning(push, 0)
#include <WebCore/WindowMessageListener.h>
#pragma warning(pop)

#include <windows.h>

class WebView;

class WebNodeHighlight : WebCore::WindowMessageListener {
public:
    WebNodeHighlight(WebView*);
    ~WebNodeHighlight();

    void setShowsWhileWebViewIsVisible(bool);

    bool isShowing() const;

    void update();

    void placeBehindWindow(HWND);

private:
    void show();
    void hide();

    bool isWebViewVisible() const;

    virtual void windowReceivedMessage(HWND, UINT message, WPARAM, LPARAM);

    void onWebViewShowWindow(bool showing);
    void onWebViewWindowPosChanged(WINDOWPOS*);
    void onRootWindowPosChanged(WINDOWPOS*);

    WebView* m_inspectedWebView;
    HWND m_inspectedWebViewWindow;
    HWND m_overlay;
    HWND m_observedWindow;
    bool m_showsWhileWebViewIsVisible;

    friend static LRESULT CALLBACK OverlayWndProc(HWND, UINT, WPARAM, LPARAM);
};

#endif // !defined(WebNodeHighlight_h)
