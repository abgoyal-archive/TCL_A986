

#include <WebCore/ChromeClient.h>
#include <WebCore/COMPtr.h>
#include <WebCore/GraphicsContext.h>
#include <WebCore/FocusDirection.h>
#include <WebCore/ScrollTypes.h>
#include <wtf/Forward.h>
#include <wtf/PassRefPtr.h>

class WebView;
class WebDesktopNotificationsDelegate;

interface IWebUIDelegate;

class WebChromeClient : public WebCore::ChromeClient {
public:
    WebChromeClient(WebView*);

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

    virtual WebCore::Page* createWindow(WebCore::Frame*, const WebCore::FrameLoadRequest&, const WebCore::WindowFeatures&);
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

    virtual void addMessageToConsole(WebCore::MessageSource source, WebCore::MessageType type, WebCore::MessageLevel level, const WebCore::String& message, unsigned line, const WebCore::String& url);

    virtual bool canRunBeforeUnloadConfirmPanel();
    virtual bool runBeforeUnloadConfirmPanel(const WebCore::String& message, WebCore::Frame* frame);

    virtual void closeWindowSoon();

    virtual void runJavaScriptAlert(WebCore::Frame*, const WebCore::String&);
    virtual bool runJavaScriptConfirm(WebCore::Frame*, const WebCore::String&);
    virtual bool runJavaScriptPrompt(WebCore::Frame*, const WebCore::String& message, const WebCore::String& defaultValue, WebCore::String& result);
    virtual void setStatusbarText(const WebCore::String&);
    virtual bool shouldInterruptJavaScript();

    virtual bool tabsToLinks() const;
    virtual WebCore::IntRect windowResizerRect() const;

    virtual void repaint(const WebCore::IntRect&, bool contentChanged, bool immediate = false, bool repaintContentOnly = false);
    virtual void scroll(const WebCore::IntSize& scrollDelta, const WebCore::IntRect& rectToScroll, const WebCore::IntRect& clipRect);
    virtual WebCore::IntPoint screenToWindow(const WebCore::IntPoint& p) const;
    virtual WebCore::IntRect windowToScreen(const WebCore::IntRect& r) const;
    virtual PlatformPageClient platformPageClient() const;
    virtual void contentsSizeChanged(WebCore::Frame*, const WebCore::IntSize&) const;

    virtual void scrollbarsModeDidChange() const { }
    virtual void mouseDidMoveOverElement(const WebCore::HitTestResult&, unsigned modifierFlags);

    virtual void setToolTip(const WebCore::String&, WebCore::TextDirection);

    virtual void print(WebCore::Frame*);

#if ENABLE(DATABASE)
    virtual void exceededDatabaseQuota(WebCore::Frame*, const WebCore::String&);
#endif

#if ENABLE(OFFLINE_WEB_APPLICATIONS)
    virtual void reachedMaxAppCacheSize(int64_t spaceNeeded);
#endif

    virtual void populateVisitedLinks();

    virtual bool paintCustomScrollbar(WebCore::GraphicsContext*, const WebCore::FloatRect&, WebCore::ScrollbarControlSize, 
                                        WebCore::ScrollbarControlState, WebCore::ScrollbarPart pressedPart, bool vertical,
                                        float value, float proportion, WebCore::ScrollbarControlPartMask);
    virtual bool paintCustomScrollCorner(WebCore::GraphicsContext*, const WebCore::FloatRect&);

    virtual void runOpenPanel(WebCore::Frame*, PassRefPtr<WebCore::FileChooser>);

    virtual bool setCursor(WebCore::PlatformCursorHandle cursor);

    WebView* webView() const { return m_webView; }

    virtual void formStateDidChange(const WebCore::Node*) { }

    virtual PassOwnPtr<WebCore::HTMLParserQuirks> createHTMLParserQuirks() { return 0; }

#if USE(ACCELERATED_COMPOSITING)
        // Pass 0 as the GraphicsLayer to detatch the root layer.
        virtual void attachRootGraphicsLayer(WebCore::Frame*, WebCore::GraphicsLayer*);
        // Sets a flag to specify that the next time content is drawn to the window,
        // the changes appear on the screen in synchrony with updates to GraphicsLayers.
        virtual void setNeedsOneShotDrawingSynchronization() { }
        // Sets a flag to specify that the view needs to be updated, so we need
        // to do an eager layout before the drawing.
        virtual void scheduleCompositingLayerSync();
#endif

    virtual void scrollRectIntoView(const WebCore::IntRect&, const WebCore::ScrollView*) const {}

    virtual void requestGeolocationPermissionForFrame(WebCore::Frame*, WebCore::Geolocation*);

#if ENABLE(VIDEO)
    virtual bool supportsFullscreenForNode(const WebCore::Node*);
    virtual void enterFullscreenForNode(WebCore::Node*);
    virtual void exitFullscreenForNode(WebCore::Node*);
#endif

#if ENABLE(NOTIFICATIONS)
    virtual WebCore::NotificationPresenter* notificationPresenter() const { return reinterpret_cast<WebCore::NotificationPresenter*>(m_notificationsDelegate.get()); }
#endif

private:
    COMPtr<IWebUIDelegate> uiDelegate();

    WebView* m_webView;

#if ENABLE(NOTIFICATIONS)
    OwnPtr<WebDesktopNotificationsDelegate> m_notificationsDelegate;
#endif
};
