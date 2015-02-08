

#ifndef ChromeClientHaiku_h
#define ChromeClientHaiku_h

#include "ChromeClient.h"
#include "FloatRect.h"
#include "RefCounted.h"

namespace WebCore {

    class FloatRect;
    class Page;
    struct FrameLoadRequest;

    class ChromeClientHaiku : public ChromeClient {
    public:
        ChromeClientHaiku();
        virtual ~ChromeClientHaiku();
        void chromeDestroyed();

        void setWindowRect(const FloatRect&);
        FloatRect windowRect();

        FloatRect pageRect();

        float scaleFactor();

        void focus();
        void unfocus();

        bool canTakeFocus(FocusDirection);
        void takeFocus(FocusDirection);

        void focusedNodeChanged(Node*);

        Page* createWindow(Frame*, const FrameLoadRequest&, const WebCore::WindowFeatures&);
        Page* createModalDialog(Frame*, const FrameLoadRequest&);
        void show();

        bool canRunModal();
        void runModal();

        void setToolbarsVisible(bool);
        bool toolbarsVisible();

        void setStatusbarVisible(bool);
        bool statusbarVisible();

        void setScrollbarsVisible(bool);
        bool scrollbarsVisible();

        void setMenubarVisible(bool);
        bool menubarVisible();

        void setResizable(bool);

        void addMessageToConsole(const String& message, unsigned int lineNumber,
                                 const String& sourceID);
        void addMessageToConsole(MessageSource, MessageLevel, const String& message,
                                 unsigned int lineNumber, const String& sourceID);
        void addMessageToConsole(MessageSource, MessageType, MessageLevel,
                                 const String&, unsigned int, const String&);

        bool canRunBeforeUnloadConfirmPanel();

        bool runBeforeUnloadConfirmPanel(const String& message, Frame* frame);

        void closeWindowSoon();

        void runJavaScriptAlert(Frame*, const String&);
        bool runJavaScriptConfirm(Frame*, const String&);
        bool runJavaScriptPrompt(Frame*, const String& message, const String& defaultValue, String& result);
        bool shouldInterruptJavaScript();

        void setStatusbarText(const WebCore::String&);
        bool tabsToLinks() const;
        IntRect windowResizerRect() const;

        void repaint(const IntRect&, bool contentChanged, bool immediate = false, bool repaintContentOnly = false);
        void scroll(const IntSize& scrollDelta, const IntRect& rectToScroll, const IntRect& clipRect);
        IntPoint screenToWindow(const IntPoint&) const;
        IntRect windowToScreen(const IntRect&) const;
        PlatformPageClient platformPageClient() const;
        void contentsSizeChanged(Frame*, const IntSize&) const;
        void scrollRectIntoView(const IntRect&, const ScrollView*) const;

        void addToDirtyRegion(const IntRect&);
        void scrollBackingStore(int, int, const IntRect&, const IntRect&);
        void updateBackingStore();

        void scrollbarsModeDidChange() const { }
        void mouseDidMoveOverElement(const HitTestResult&, unsigned modifierFlags);

        void setToolTip(const String&);

        virtual void setToolTip(const String&, TextDirection);

        void print(Frame*);

        void exceededDatabaseQuota(Frame*, const String& databaseName);

#if ENABLE(OFFLINE_WEB_APPLICATIONS)
        virtual void reachedMaxAppCacheSize(int64_t spaceNeeded);
#endif

        // This is an asynchronous call. The ChromeClient can display UI asking the user for permission
        // to use Geolococation. The ChromeClient must call Geolocation::setShouldClearCache() appropriately.
        void requestGeolocationPermissionForFrame(Frame*, Geolocation*);

        void runOpenPanel(Frame*, PassRefPtr<FileChooser>);

        bool setCursor(PlatformCursorHandle);

        // Notification that the given form element has changed. This function
        // will be called frequently, so handling should be very fast.
        void formStateDidChange(const Node*);

        PassOwnPtr<HTMLParserQuirks> createHTMLParserQuirks();
    };

} // namespace WebCore

#endif

