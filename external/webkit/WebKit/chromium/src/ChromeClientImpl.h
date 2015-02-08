

#ifndef ChromeClientImpl_h
#define ChromeClientImpl_h

#include "ChromeClientChromium.h"

namespace WebCore {
class AccessibilityObject;
class HTMLParserQuirks;
class PopupContainer;
class SecurityOrigin;
struct WindowFeatures;
}

namespace WebKit {
class WebViewImpl;
struct WebCursorInfo;
struct WebPopupMenuInfo;

// Handles window-level notifications from WebCore on behalf of a WebView.
class ChromeClientImpl : public WebCore::ChromeClientChromium {
public:
    explicit ChromeClientImpl(WebViewImpl* webView);
    virtual ~ChromeClientImpl();

    WebViewImpl* webView() const { return m_webView; }

    // ChromeClient methods:
    virtual void chromeDestroyed();
    virtual void setWindowRect(const WebCore::FloatRect&);
    virtual WebCore::FloatRect windowRect();
    virtual WebCore::FloatRect pageRect();
    virtual float scaleFactor();
    virtual void focus();
    virtual void unfocus();
    virtual bool canTakeFocus(WebCore::FocusDirection);
    virtual void takeFocus(WebCore::FocusDirection);
    virtual void focusedNodeChanged(WebCore::Node*);
    virtual WebCore::Page* createWindow(
        WebCore::Frame*, const WebCore::FrameLoadRequest&, const WebCore::WindowFeatures&);
    virtual void show();
    virtual bool canRunModal();
    virtual void runModal();
    virtual void setToolbarsVisible(bool);
    virtual bool toolbarsVisible();
    virtual void setStatusbarVisible(bool);
    virtual bool statusbarVisible();
    virtual void setScrollbarsVisible(bool);
    virtual bool scrollbarsVisible();
    virtual void setMenubarVisible(bool);
    virtual bool menubarVisible();
    virtual void setResizable(bool);
    virtual void addMessageToConsole(
        WebCore::MessageSource, WebCore::MessageType, WebCore::MessageLevel,
        const WebCore::String& message, unsigned lineNumber,
        const WebCore::String& sourceID);
    virtual bool canRunBeforeUnloadConfirmPanel();
    virtual bool runBeforeUnloadConfirmPanel(
        const WebCore::String& message, WebCore::Frame*);
    virtual void closeWindowSoon();
    virtual void runJavaScriptAlert(WebCore::Frame*, const WebCore::String&);
    virtual bool runJavaScriptConfirm(WebCore::Frame*, const WebCore::String&);
    virtual bool runJavaScriptPrompt(
        WebCore::Frame*, const WebCore::String& message,
        const WebCore::String& defaultValue, WebCore::String& result);
    virtual void setStatusbarText(const WebCore::String& message);
    virtual bool shouldInterruptJavaScript();
    virtual bool tabsToLinks() const;
    virtual WebCore::IntRect windowResizerRect() const;
    virtual void repaint(
        const WebCore::IntRect&, bool contentChanged, bool immediate = false,
        bool repaintContentOnly = false);
    virtual void scroll(
        const WebCore::IntSize& scrollDelta, const WebCore::IntRect& rectToScroll,
        const WebCore::IntRect& clipRect);
    virtual WebCore::IntPoint screenToWindow(const WebCore::IntPoint&) const;
    virtual WebCore::IntRect windowToScreen(const WebCore::IntRect&) const;
    virtual PlatformPageClient platformPageClient() const { return 0; }
    virtual void contentsSizeChanged(WebCore::Frame*, const WebCore::IntSize&) const;
    virtual void scrollRectIntoView(
        const WebCore::IntRect&, const WebCore::ScrollView*) const { }
    virtual void scrollbarsModeDidChange() const;
    virtual void mouseDidMoveOverElement(
        const WebCore::HitTestResult& result, unsigned modifierFlags);
    virtual void setToolTip(const WebCore::String& tooltipText, WebCore::TextDirection);
    virtual void print(WebCore::Frame*);
    virtual void exceededDatabaseQuota(
        WebCore::Frame*, const WebCore::String& databaseName);
#if ENABLE(OFFLINE_WEB_APPLICATIONS)
    virtual void reachedMaxAppCacheSize(int64_t spaceNeeded);
#endif
#if ENABLE(NOTIFICATIONS)
    virtual WebCore::NotificationPresenter* notificationPresenter() const;
#endif
    virtual void requestGeolocationPermissionForFrame(
        WebCore::Frame*, WebCore::Geolocation*) { }
    virtual void runOpenPanel(WebCore::Frame*, PassRefPtr<WebCore::FileChooser>);
    virtual bool setCursor(WebCore::PlatformCursorHandle) { return false; }
    virtual void formStateDidChange(const WebCore::Node*);
    virtual PassOwnPtr<WebCore::HTMLParserQuirks> createHTMLParserQuirks() { return 0; }

    // ChromeClientChromium methods:
    virtual void popupOpened(WebCore::PopupContainer* popupContainer,
                             const WebCore::IntRect& bounds,
                             bool activatable,
                             bool handleExternally);
    virtual void didChangeAccessibilityObjectState(WebCore::AccessibilityObject*);

    // ChromeClientImpl:
    void setCursor(const WebCursorInfo& cursor);
    void setCursorForPlugin(const WebCursorInfo& cursor);

private:
    void getPopupMenuInfo(WebCore::PopupContainer*, WebPopupMenuInfo*);

    WebViewImpl* m_webView;  // weak pointer
    bool m_toolbarsVisible;
    bool m_statusbarVisible;
    bool m_scrollbarsVisible;
    bool m_menubarVisible;
    bool m_resizable;
    // Set to true if the next SetCursor is to be ignored.
    bool m_ignoreNextSetCursor;
};

} // namespace WebKit

#endif
