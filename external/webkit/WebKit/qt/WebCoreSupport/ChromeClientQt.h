

#ifndef ChromeClientQt_H
#define ChromeClientQt_H

#include "ChromeClient.h"
#include "FloatRect.h"
#include "RefCounted.h"
#include "KURL.h"
#include "PlatformString.h"

class QEventLoop;
class QWebPage;

namespace WebCore {

    class FileChooser;
    class FloatRect;
    class Page;
    struct FrameLoadRequest;
    class QtAbstractWebPopup;

    class ChromeClientQt : public ChromeClient
    {
    public:
        ChromeClientQt(QWebPage* webPage);
        virtual ~ChromeClientQt();
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

        virtual void addMessageToConsole(MessageSource, MessageType, MessageLevel, const String& message,
                                         unsigned int lineNumber, const String& sourceID);

        virtual bool canRunBeforeUnloadConfirmPanel();
        virtual bool runBeforeUnloadConfirmPanel(const String& message, Frame* frame);

        virtual void closeWindowSoon();

        virtual void runJavaScriptAlert(Frame*, const String&);
        virtual bool runJavaScriptConfirm(Frame*, const String&);
        virtual bool runJavaScriptPrompt(Frame*, const String& message, const String& defaultValue, String& result);
        virtual bool shouldInterruptJavaScript();

        virtual void setStatusbarText(const String&);

        virtual bool tabsToLinks() const;
        virtual IntRect windowResizerRect() const;

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

#if USE(ACCELERATED_COMPOSITING)
        // see ChromeClient.h
        // this is a hook for WebCore to tell us what we need to do with the GraphicsLayers
        virtual void attachRootGraphicsLayer(Frame*, GraphicsLayer*);
        virtual void setNeedsOneShotDrawingSynchronization();
        virtual void scheduleCompositingLayerSync();
#endif

#if ENABLE(TOUCH_EVENTS)
        virtual void needTouchEvents(bool) { }
#endif

        virtual void runOpenPanel(Frame*, PassRefPtr<FileChooser>);

        virtual void formStateDidChange(const Node*) { }

        virtual PassOwnPtr<HTMLParserQuirks> createHTMLParserQuirks() { return 0; }

        virtual bool setCursor(PlatformCursorHandle);

        virtual void scrollRectIntoView(const IntRect&, const ScrollView*) const {}

        virtual void requestGeolocationPermissionForFrame(Frame*, Geolocation*);

        QtAbstractWebPopup* createSelectPopup();

        QWebPage* m_webPage;
        WebCore::KURL lastHoverURL;
        WebCore::String lastHoverTitle;
        WebCore::String lastHoverContent;

        bool toolBarsVisible;
        bool statusBarVisible;
        bool menuBarVisible;
        QEventLoop* m_eventLoop;
    };
}

#endif
