

#import <WebCore/ChromeClient.h>
#import <WebCore/FocusDirection.h>
#import <wtf/Forward.h>

@class WebView;

class WebChromeClient : public WebCore::ChromeClient {
public:
    WebChromeClient(WebView *webView);
    WebView *webView() { return m_webView; }
    
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
    
    virtual void addMessageToConsole(WebCore::MessageSource source, WebCore::MessageType type, WebCore::MessageLevel level, const WebCore::String& message, unsigned int lineNumber, const WebCore::String& sourceURL);

    virtual bool canRunBeforeUnloadConfirmPanel();
    virtual bool runBeforeUnloadConfirmPanel(const WebCore::String& message, WebCore::Frame* frame);

    virtual void closeWindowSoon();

    virtual void runJavaScriptAlert(WebCore::Frame*, const WebCore::String&);
    virtual bool runJavaScriptConfirm(WebCore::Frame*, const WebCore::String&);
    virtual bool runJavaScriptPrompt(WebCore::Frame*, const WebCore::String& message, const WebCore::String& defaultValue, WebCore::String& result);
    virtual bool shouldInterruptJavaScript();

    virtual bool tabsToLinks() const;
    
    virtual WebCore::IntRect windowResizerRect() const;

    virtual void repaint(const WebCore::IntRect&, bool contentChanged, bool immediate = false, bool repaintContentOnly = false);
    virtual void scroll(const WebCore::IntSize& scrollDelta, const WebCore::IntRect& rectToScroll, const WebCore::IntRect& clipRect);
    virtual WebCore::IntPoint screenToWindow(const WebCore::IntPoint&) const;
    virtual WebCore::IntRect windowToScreen(const WebCore::IntRect&) const;
    virtual PlatformPageClient platformPageClient() const;
    virtual void contentsSizeChanged(WebCore::Frame*, const WebCore::IntSize&) const;
    virtual void scrollRectIntoView(const WebCore::IntRect&, const WebCore::ScrollView*) const;
    
    virtual void setStatusbarText(const WebCore::String&);

    virtual void scrollbarsModeDidChange() const { }
    virtual void mouseDidMoveOverElement(const WebCore::HitTestResult&, unsigned modifierFlags);

    virtual void setToolTip(const WebCore::String&, WebCore::TextDirection);

    virtual void print(WebCore::Frame*);
#if ENABLE(DATABASE)
    virtual void exceededDatabaseQuota(WebCore::Frame*, const WebCore::String& databaseName);
#endif
#if ENABLE(OFFLINE_WEB_APPLICATIONS)
    virtual void reachedMaxAppCacheSize(int64_t spaceNeeded);
#endif
    virtual void populateVisitedLinks();

#if ENABLE(DASHBOARD_SUPPORT)
    virtual void dashboardRegionsChanged();
#endif

    virtual void runOpenPanel(WebCore::Frame*, PassRefPtr<WebCore::FileChooser>);

    virtual bool setCursor(WebCore::PlatformCursorHandle) { return false; }

    virtual WebCore::FloatRect customHighlightRect(WebCore::Node*, const WebCore::AtomicString& type,
        const WebCore::FloatRect& lineRect);
    virtual void paintCustomHighlight(WebCore::Node*, const WebCore::AtomicString& type,
        const WebCore::FloatRect& boxRect, const WebCore::FloatRect& lineRect,
        bool behindText, bool entireLine);

    virtual WebCore::KeyboardUIMode keyboardUIMode();

    virtual NSResponder *firstResponder();
    virtual void makeFirstResponder(NSResponder *);

    virtual void willPopUpMenu(NSMenu *);
    
    virtual bool shouldReplaceWithGeneratedFileForUpload(const WebCore::String& path, WebCore::String &generatedFilename);
    virtual WebCore::String generateReplacementFile(const WebCore::String& path);

    virtual void formStateDidChange(const WebCore::Node*);

    virtual void formDidFocus(const WebCore::Node*);
    virtual void formDidBlur(const WebCore::Node*);

    virtual PassOwnPtr<WebCore::HTMLParserQuirks> createHTMLParserQuirks() { return 0; }

#if USE(ACCELERATED_COMPOSITING)
    virtual void attachRootGraphicsLayer(WebCore::Frame*, WebCore::GraphicsLayer*);
    virtual void setNeedsOneShotDrawingSynchronization();
    virtual void scheduleCompositingLayerSync();
#endif

#if ENABLE(VIDEO)
    virtual bool supportsFullscreenForNode(const WebCore::Node*);
    virtual void enterFullscreenForNode(WebCore::Node*);
    virtual void exitFullscreenForNode(WebCore::Node*);
#endif

    virtual void requestGeolocationPermissionForFrame(WebCore::Frame*, WebCore::Geolocation*);
    virtual void cancelGeolocationPermissionRequestForFrame(WebCore::Frame*) { }

private:
    WebView *m_webView;
};
