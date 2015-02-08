

#ifndef Chrome_h
#define Chrome_h

#include "Cursor.h"
#include "FileChooser.h"
#include "FocusDirection.h"
#include "HostWindow.h"
#include <wtf/Forward.h>
#include <wtf/RefPtr.h>

#if PLATFORM(MAC)
#ifndef __OBJC__
class NSView;
#endif
#endif

namespace WebCore {

    class ChromeClient;
    class ContextMenu;
    class FloatRect;
    class Frame;
    class Geolocation;
    class HitTestResult;
    class IntRect;
    class Node;
    class Page;
    class String;
#if ENABLE(NOTIFICATIONS)
    class NotificationPresenter;
#endif

    struct FrameLoadRequest;
    struct WindowFeatures;
    
    class Chrome : public HostWindow {
    public:
        Chrome(Page*, ChromeClient*);
        ~Chrome();

        ChromeClient* client() { return m_client; }

        // HostWindow methods.
        virtual void repaint(const IntRect&, bool contentChanged, bool immediate = false, bool repaintContentOnly = false);
        virtual void scroll(const IntSize& scrollDelta, const IntRect& rectToScroll, const IntRect& clipRect);
        virtual IntPoint screenToWindow(const IntPoint&) const;
        virtual IntRect windowToScreen(const IntRect&) const;
        virtual PlatformPageClient platformPageClient() const;
        virtual void scrollRectIntoView(const IntRect&, const ScrollView*) const;
        virtual void scrollbarsModeDidChange() const;

        void contentsSizeChanged(Frame*, const IntSize&) const;

        void setWindowRect(const FloatRect&) const;
        FloatRect windowRect() const;

        FloatRect pageRect() const;
        
        float scaleFactor();

#ifdef ANDROID_USER_GESTURE
        void focus(bool userGesture) const;
#else
        void focus() const;
#endif
        void unfocus() const;

        bool canTakeFocus(FocusDirection) const;
        void takeFocus(FocusDirection) const;

        void focusedNodeChanged(Node*) const;

        Page* createWindow(Frame*, const FrameLoadRequest&, const WindowFeatures&) const;
        void show() const;

        bool canRunModal() const;
        bool canRunModalNow() const;
        void runModal() const;

        void setToolbarsVisible(bool) const;
        bool toolbarsVisible() const;
        
        void setStatusbarVisible(bool) const;
        bool statusbarVisible() const;
        
        void setScrollbarsVisible(bool) const;
        bool scrollbarsVisible() const;
        
        void setMenubarVisible(bool) const;
        bool menubarVisible() const;
        
        void setResizable(bool) const;

        bool canRunBeforeUnloadConfirmPanel();
        bool runBeforeUnloadConfirmPanel(const String& message, Frame* frame);

        void closeWindowSoon();

        void runJavaScriptAlert(Frame*, const String&);
        bool runJavaScriptConfirm(Frame*, const String&);
        bool runJavaScriptPrompt(Frame*, const String& message, const String& defaultValue, String& result);
        void setStatusbarText(Frame*, const String&);
        bool shouldInterruptJavaScript();

        void registerProtocolHandler(const String& scheme, const String& baseURL, const String& url, const String& title);
        void registerContentHandler(const String& mimeType, const String& baseURL, const String& url, const String& title);

        IntRect windowResizerRect() const;

        void mouseDidMoveOverElement(const HitTestResult&, unsigned modifierFlags);

        void setToolTip(const HitTestResult&);

        void print(Frame*);

        void requestGeolocationPermissionForFrame(Frame*, Geolocation*);
        void cancelGeolocationPermissionRequestForFrame(Frame*);

        void runOpenPanel(Frame*, PassRefPtr<FileChooser>);

        bool setCursor(PlatformCursorHandle);

#if PLATFORM(MAC)
        void focusNSView(NSView*);
#endif

#if ENABLE(NOTIFICATIONS)
        NotificationPresenter* notificationPresenter() const; 
#endif

    private:
        Page* m_page;
        ChromeClient* m_client;
    };
}

#endif // Chrome_h
