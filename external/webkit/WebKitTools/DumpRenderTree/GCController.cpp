

#include "config.h"
#include "GCController.h"

#include <JavaScriptCore/JSObjectRef.h>
#include <JavaScriptCore/JSRetainPtr.h>

GCController::GCController()
{
}

GCController::~GCController()
{
}

// Static Functions

static JSValueRef collectCallback(JSContextRef context, JSObjectRef function, JSObjectRef thisObject, size_t argumentCount, const JSValueRef arguments[], JSValueRef* exception)
{
    GCController* controller = static_cast<GCController*>(JSObjectGetPrivate(thisObject));
    controller->collect();
    return JSValueMakeUndefined(context);
}

static JSValueRef collectOnAlternateThreadCallback(JSContextRef context, JSObjectRef function, JSObjectRef thisObject, size_t argumentCount, const JSValueRef arguments[], JSValueRef* exception)
{
    bool waitUntilDone = false;
    if (argumentCount > 0)
        waitUntilDone = JSValueToBoolean(context, arguments[0]);

    GCController* controller = static_cast<GCController*>(JSObjectGetPrivate(thisObject));
    controller->collectOnAlternateThread(waitUntilDone);

    return JSValueMakeUndefined(context);
}

static JSValueRef getJSObjectCountCallback(JSContextRef context, JSObjectRef function, JSObjectRef thisObject, size_t argumentCount, const JSValueRef arguments[], JSValueRef* exception)
{
    GCController* controller = static_cast<GCController*>(JSObjectGetPrivate(thisObject));
    size_t jsObjectCount = controller->getJSObjectCount();

    return JSValueMakeNumber(context, jsObjectCount);
}

// Object Creation

void GCController::makeWindowObject(JSContextRef context, JSObjectRef windowObject, JSValueRef* exception)
{
    JSRetainPtr<JSStringRef> gcControllerStr(Adopt, JSStringCreateWithUTF8CString("GCController"));

    JSClassRef classRef = getJSClass();
    JSValueRef gcControllerObject = JSObjectMake(context, classRef, this);
    JSClassRelease(classRef);

    JSObjectSetProperty(context, windowObject, gcControllerStr.get(), gcControllerObject, kJSPropertyAttributeReadOnly | kJSPropertyAttributeDontDelete, exception);
}

JSClassRef GCController::getJSClass()
{
    static JSStaticFunction staticFunctions[] = {
        { "collect", collectCallback, kJSPropertyAttributeReadOnly | kJSPropertyAttributeDontDelete },
        { "collectOnAlternateThread", collectOnAlternateThreadCallback, kJSPropertyAttributeReadOnly | kJSPropertyAttributeDontDelete },
        { "getJSObjectCount", getJSObjectCountCallback, kJSPropertyAttributeReadOnly | kJSPropertyAttributeDontDelete },
        { 0, 0, 0 }
    };

    static JSClassDefinition classDefinition = {
        0, kJSClassAttributeNone, "GCController", 0, 0, staticFunctions,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    return JSClassCreate(&classDefinition);
}
