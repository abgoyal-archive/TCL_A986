

#ifndef ChromeClientWx_H
#define ChromeClientWx_H

#include "ChromeClient.h"
#include "FocusDirection.h"
#include "IntRect.h"
#include "WebView.h"

namespace WebCore {

class ChromeClientWx : public ChromeClient {
public:
    ChromeClientWx(wxWebView*);
    virtual ~ChromeClientWx();
    virtual void chromeDestroyed();

    virtual void setWindowRect(const FloatRect&);
    virtual FloatRect windowRect();

    virtual FloatRect pageRect();

    virtual float scaleFactor();

    virtual void focus();
    virtual void unfocus();

    virtual bool canTakeFocus(FocusDirection);
    virtual void takeFocus(FocusDirection);

    virtual void focusedNodeChanged(Node*);

    virtual Page* createWindow(Frame*, const FrameLoadRequest&, const WindowFeatures&);
    virtual Page* createModalDialog(Frame*, const FrameLoadRequest&);
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

    virtual void addMessageToConsole(MessageSource source,
                                     MessageType type,
                                     MessageLevel level,
                                     const String& message,
                                     unsigned int lineNumber,
                                     const String& sourceID);

    virtual bool canRunBeforeUnloadConfirmPanel();
    virtual bool runBeforeUnloadConfirmPanel(const String& message,
                                             Frame* frame);

    virtual void closeWindowSoon();
    
    virtual void runJavaScriptAlert(Frame*, const String&);
    virtual bool runJavaScriptConfirm(Frame*, const String&);
    virtual bool runJavaScriptPrompt(Frame*, const String& message, const String& defaultValue, String& result);
    virtual void setStatusbarText(const String&);
    virtual bool shouldInterruptJavaScript();
    
    virtual bool tabsToLinks() const;

    virtual IntRect windowResizerRect() const;
    virtual void scrollBackingStore(int dx, int dy, const IntRect& scrollViewRect, const IntRect& clipRect);
    virtual void updateBackingStore();
    
    virtual void repaint(const IntRect&, bool contentChanged, bool immediate = false, bool repaintContentOnly = false);
    virtual void scroll(const IntSize& scrollDelta, const IntRect& rectToScroll, const IntRect& clipRect);
    virtual IntPoint screenToWindow(const IntPoint&) const;
    virtual IntRect windowToScreen(const IntRect&) const;
    virtual PlatformPageClient platformPageClient() const;
    virtual void contentsSizeChanged(Frame*, const IntSize&) const;

    virtual void scrollbarsModeDidChange() const { }
    virtual void mouseDidMoveOverElement(const HitTestResult&, unsigned modifierFlags);

    virtual void setToolTip(const String&, TextDirection);

    virtual void print(Frame*);

#if ENABLE(DATABASE)
    virtual void exceededDatabaseQuota(Frame*, const String&);
#endif

#if ENABLE(OFFLINE_WEB_APPLICATIONS)
    virtual void reachedMaxAppCacheSize(int64_t spaceNeeded);
#endif

    virtual void runOpenPanel(Frame*, PassRefPtr<FileChooser>);

    virtual void formStateDidChange(const Node*) { }

    virtual PassOwnPtr<HTMLParserQuirks> createHTMLParserQuirks() { return 0; }

    virtual bool setCursor(PlatformCursorHandle);

    virtual void scrollRectIntoView(const IntRect&, const ScrollView*) const {}

    virtual void requestGeolocationPermissionForFrame(Frame*, Geolocation*);

private:
    wxWebView* m_webView;
};

}
#endif // ChromeClientWx_H
