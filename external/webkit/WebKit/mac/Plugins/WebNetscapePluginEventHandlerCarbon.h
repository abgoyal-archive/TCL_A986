

#if ENABLE(NETSCAPE_PLUGIN_API) && !defined(__LP64__)

#ifndef WebNetscapePluginEventHandlerCarbon_h
#define WebNetscapePluginEventHandlerCarbon_h

#include "WebNetscapePluginEventHandler.h"

#import <Carbon/Carbon.h>
#import <wtf/RetainPtr.h>

class WebNetscapePluginEventHandlerCarbon : public WebNetscapePluginEventHandler {
public:
    WebNetscapePluginEventHandlerCarbon(WebNetscapePluginView*); 

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

    virtual void startTimers(bool throttleTimers);
    virtual void stopTimers();

    virtual void* platformWindow(NSWindow*);
    
private:
    void sendNullEvent();

    void installKeyEventHandler();
    void removeKeyEventHandler();
    
    static OSStatus TSMEventHandler(EventHandlerCallRef, EventRef, void *eventHandler);
    static void nullEventTimerFired(CFRunLoopTimerRef, void *context);

    bool sendEvent(EventRecord*);
    
    EventHandlerRef m_keyEventHandler;
    bool m_suspendKeyUpEvents;
    RetainPtr<CFRunLoopTimerRef> m_nullEventTimer;
};

#endif // WebNetscapePluginEventHandlerCarbon_h

#endif // ENABLE(NETSCAPE_PLUGIN_API) && !defined(__LP64__)

