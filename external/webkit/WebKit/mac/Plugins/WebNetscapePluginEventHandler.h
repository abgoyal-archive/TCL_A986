

#ifndef WebNetscapePluginEventHandler_h
#define WebNetscapePluginEventHandler_h

#import "WebNetscapePluginView.h"

#if ENABLE(NETSCAPE_PLUGIN_API)

@class NSEvent;
@class WebNetscapePluginView;

struct CGRect;

class WebNetscapePluginEventHandler {
public:
    static WebNetscapePluginEventHandler* create(WebNetscapePluginView*);
    virtual ~WebNetscapePluginEventHandler() { }
    
    virtual void drawRect(CGContextRef, const NSRect&) = 0;
    
    virtual void mouseDown(NSEvent*) = 0;
    virtual void mouseDragged(NSEvent*) = 0;
    virtual void mouseEntered(NSEvent*) = 0;
    virtual void mouseExited(NSEvent*) = 0;
    virtual void mouseMoved(NSEvent*) = 0;
    virtual void mouseUp(NSEvent*) = 0;
    virtual bool scrollWheel(NSEvent*) = 0;
    
    virtual void keyDown(NSEvent*) = 0;
    virtual void keyUp(NSEvent*) = 0;
    virtual void flagsChanged(NSEvent*) = 0;
    virtual void syntheticKeyDownWithCommandModifier(int keyCode, char character) = 0;
    
    virtual void focusChanged(bool hasFocus) = 0;
    virtual void windowFocusChanged(bool hasFocus) = 0;
    
    virtual void startTimers(bool throttleTimers) { }
    virtual void stopTimers() { }
    
    // Returns the platform specific window used in NPP_SetWindow
    virtual void* platformWindow(NSWindow*) = 0;
    
    bool currentEventIsUserGesture() const { return m_currentEventIsUserGesture; }
protected:
    WebNetscapePluginEventHandler(WebNetscapePluginView* pluginView)
        : m_pluginView(pluginView)
        , m_currentEventIsUserGesture(false)
    {
    }
    
    WebNetscapePluginView* m_pluginView;
    bool m_currentEventIsUserGesture;
};

#endif // ENABLE(NETSCAPE_PLUGIN_API)

#endif // WebNetscapePluginEventHandler_h


