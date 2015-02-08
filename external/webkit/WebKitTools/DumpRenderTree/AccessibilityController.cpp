

#include "config.h"
#include "AccessibilityController.h"

#include "AccessibilityUIElement.h"
#include <JavaScriptCore/JSRetainPtr.h>

// Static Value Getters

static JSValueRef getFocusedElementCallback(JSContextRef context, JSObjectRef thisObject, JSStringRef propertyName, JSValueRef* exception)
{
    AccessibilityController* controller = static_cast<AccessibilityController*>(JSObjectGetPrivate(thisObject));
    return AccessibilityUIElement::makeJSAccessibilityUIElement(context, controller->focusedElement());
}

static JSValueRef getRootElementCallback(JSContextRef context, JSObjectRef thisObject, JSStringRef propertyName, JSValueRef* exception)
{
    AccessibilityController* controller = static_cast<AccessibilityController*>(JSObjectGetPrivate(thisObject));
    return AccessibilityUIElement::makeJSAccessibilityUIElement(context, controller->rootElement());
}

// Object Creation

void AccessibilityController::makeWindowObject(JSContextRef context, JSObjectRef windowObject, JSValueRef* exception)
{
    JSRetainPtr<JSStringRef> accessibilityControllerStr(Adopt, JSStringCreateWithUTF8CString("accessibilityController"));
    
    JSClassRef classRef = getJSClass();
    JSValueRef accessibilityControllerObject = JSObjectMake(context, classRef, this);
    JSClassRelease(classRef);

    JSObjectSetProperty(context, windowObject, accessibilityControllerStr.get(), accessibilityControllerObject, kJSPropertyAttributeReadOnly | kJSPropertyAttributeDontDelete, exception);
}

static JSValueRef logFocusEventsCallback(JSContextRef ctx, JSObjectRef, JSObjectRef thisObject, size_t, const JSValueRef[], JSValueRef*)
{
    AccessibilityController* controller = static_cast<AccessibilityController*>(JSObjectGetPrivate(thisObject));
    controller->setLogFocusEvents(true);
    return JSValueMakeUndefined(ctx);
}

static JSValueRef logValueChangeEventsCallback(JSContextRef ctx, JSObjectRef, JSObjectRef thisObject, size_t, const JSValueRef[], JSValueRef*)
{
    AccessibilityController* controller = static_cast<AccessibilityController*>(JSObjectGetPrivate(thisObject));
    controller->setLogValueChangeEvents(true);
    return JSValueMakeUndefined(ctx);
}

static JSValueRef logScrollingStartEventsCallback(JSContextRef ctx, JSObjectRef, JSObjectRef thisObject, size_t, const JSValueRef[], JSValueRef*)
{
    AccessibilityController* controller = static_cast<AccessibilityController*>(JSObjectGetPrivate(thisObject));
    controller->setLogScrollingStartEvents(true);
    return JSValueMakeUndefined(ctx);
}

JSClassRef AccessibilityController::getJSClass()
{
    static JSStaticFunction staticFunctions[] = {
        { "logFocusEvents", logFocusEventsCallback, kJSPropertyAttributeReadOnly | kJSPropertyAttributeDontDelete },
        { "logValueChangeEvents", logValueChangeEventsCallback, kJSPropertyAttributeReadOnly | kJSPropertyAttributeDontDelete },
        { "logScrollingStartEvents", logScrollingStartEventsCallback, kJSPropertyAttributeReadOnly | kJSPropertyAttributeDontDelete },
        { 0, 0, 0 }
    };

    static JSStaticValue staticValues[] = {
        { "focusedElement", getFocusedElementCallback, 0, kJSPropertyAttributeReadOnly | kJSPropertyAttributeDontDelete },
        { "rootElement", getRootElementCallback, 0, kJSPropertyAttributeReadOnly | kJSPropertyAttributeDontDelete },
        { 0, 0, 0, 0 }
    };

    static JSClassDefinition classDefinition = {
        0, kJSClassAttributeNone, "AccessibilityController", 0, staticValues, staticFunctions,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    return JSClassCreate(&classDefinition);
}

void AccessibilityController::resetToConsistentState()
{
    setLogFocusEvents(false);
    setLogValueChangeEvents(false);
    setLogScrollingStartEvents(false);
}
