

#ifndef AccessibilityController_h
#define AccessibilityController_h

#include "AccessibilityUIElement.h"
#include <JavaScriptCore/JSObjectRef.h>
#include <string>
#include <wtf/HashMap.h>
#include <wtf/Platform.h>
#if PLATFORM(WIN)
#include <windows.h>
#endif

class AccessibilityController {
public:
    AccessibilityController();
    ~AccessibilityController();

    void makeWindowObject(JSContextRef context, JSObjectRef windowObject, JSValueRef* exception);

    // Controller Methods - platform-independent implementations
    AccessibilityUIElement rootElement();
    AccessibilityUIElement focusedElement();

    void setLogFocusEvents(bool);
    void setLogValueChangeEvents(bool);
    void setLogScrollingStartEvents(bool);

    void resetToConsistentState();

    void addNotificationListener(PlatformUIElement, JSObjectRef functionCallback);
    void notificationReceived(PlatformUIElement, const std::string& eventName);

private:
    static JSClassRef getJSClass();

#if PLATFORM(WIN)
    HWINEVENTHOOK m_focusEventHook;
    HWINEVENTHOOK m_valueChangeEventHook;
    HWINEVENTHOOK m_scrollingStartEventHook;

    HWINEVENTHOOK m_allEventsHook;
    HashMap<PlatformUIElement, JSObjectRef> m_notificationListeners;
#endif
};

#endif // AccessibilityController_h
