

#if ENABLE(NETSCAPE_PLUGIN_API)

#ifndef WebNetscapePluginEventHandlerCocoa_h
#define WebNetscapePluginEventHandlerCocoa_h

#include <WebKit/npapi.h>
#include "WebNetscapePluginEventHandler.h"

class WebNetscapePluginEventHandlerCocoa : public WebNetscapePluginEventHandler {
public:
    WebNetscapePluginEventHandlerCocoa(WebNetscapePluginView*); 

    virtual void drawRect(CGContextRef, const NSRect&);

    virtual void mouseDown(NSEvent*);
    virtual void mouseDragged(NSEvent*);
    virtual void mouseEntered(NSEvent*);
    virtual void mouseExited(NSEvent*);
    virtual void mouseMoved(NSEvent*);
    virtual void mouseUp(NSEvent*);
    virtual bool scrollWheel(NSEvent*);
    
    virtual void keyDown(NSEvent*);
    virtual void keyUp(NSEvent*);
    virtual void flagsChanged(NSEvent*);
    virtual void syntheticKeyDownWithCommandModifier(int keyCode, char character);

    virtual void windowFocusChanged(bool hasFocus);    
    virtual void focusChanged(bool hasFocus);

    virtual void* platformWindow(NSWindow*);
    
private:
    bool sendMouseEvent(NSEvent*, NPCocoaEventType);
    bool sendKeyEvent(NSEvent*, NPCocoaEventType);
    bool sendEvent(NPCocoaEvent*);
    
#ifndef __LP64__
    void installKeyEventHandler();
    void removeKeyEventHandler();
    
    static OSStatus TSMEventHandler(EventHandlerCallRef, EventRef, void *eventHandler);
    OSStatus handleTSMEvent(EventRef);

    EventHandlerRef m_keyEventHandler;
#else
    inline void installKeyEventHandler() { }
    void removeKeyEventHandler() { }
#endif
};

#endif //WebNetscapePluginEventHandlerCocoa_h

#endif // ENABLE(NETSCAPE_PLUGIN_API)

