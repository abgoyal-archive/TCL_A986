

#include "config.h"
#include "JavaScriptProfile.h"

#if ENABLE(JAVASCRIPT_DEBUGGER) && USE(JSC)

#include "JavaScriptProfileNode.h"
#include <profiler/Profile.h>
#include <JavaScriptCore/APICast.h>
#include <JavaScriptCore/JSObjectRef.h>
#include <JavaScriptCore/JSStringRef.h>
#include <JavaScriptCore/OpaqueJSString.h>
#include <runtime/JSObject.h>
#include <runtime/JSValue.h>
#include <wtf/StdLibExtras.h>

using namespace JSC;

namespace WebCore {

// Cache

typedef HashMap<Profile*, JSObject*> ProfileMap;

static ProfileMap& profileCache()
{ 
    DEFINE_STATIC_LOCAL(ProfileMap, staticProfiles, ());
    return staticProfiles;
}

// Static Values

static JSClassRef ProfileClass();

static JSValueRef getTitleCallback(JSContextRef ctx, JSObjectRef thisObject, JSStringRef, JSValueRef*)
{
    if (!JSValueIsObjectOfClass(ctx, thisObject, ProfileClass()))
        return JSValueMakeUndefined(ctx);

    Profile* profile = static_cast<Profile*>(JSObjectGetPrivate(thisObject));
    return JSValueMakeString(ctx, OpaqueJSString::create(profile->title()).get());
}

static JSValueRef getHeadCallback(JSContextRef ctx, JSObjectRef thisObject, JSStringRef, JSValueRef*)
{
    if (!JSValueIsObjectOfClass(ctx, thisObject, ProfileClass()))
        return JSValueMakeUndefined(ctx);

    ExecState* exec = toJS(ctx);
    Profile* profile = static_cast<Profile*>(JSObjectGetPrivate(thisObject));
    return toRef(exec, toJS(exec, profile->head()));
}

static JSValueRef getUniqueIdCallback(JSContextRef ctx, JSObjectRef thisObject, JSStringRef, JSValueRef*)
{
    if (!JSValueIsObjectOfClass(ctx, thisObject, ProfileClass()))
        return JSValueMakeUndefined(ctx);

    Profile* profile = static_cast<Profile*>(JSObjectGetPrivate(thisObject));
    return JSValueMakeNumber(ctx, profile->uid());
}

// Static Functions

static JSValueRef focus(JSContextRef ctx, JSObjectRef /*function*/, JSObjectRef thisObject, size_t argumentCount, const JSValueRef arguments[], JSValueRef* /*exception*/)
{
    if (!JSValueIsObjectOfClass(ctx, thisObject, ProfileClass()))
        return JSValueMakeUndefined(ctx);

    if (argumentCount < 1)
        return JSValueMakeUndefined(ctx);

    if (!JSValueIsObjectOfClass(ctx, arguments[0], ProfileNodeClass()))
        return JSValueMakeUndefined(ctx);

    Profile* profile = static_cast<Profile*>(JSObjectGetPrivate(thisObject));
    profile->focus(static_cast<ProfileNode*>(JSObjectGetPrivate(const_cast<JSObjectRef>(arguments[0]))));

    return JSValueMakeUndefined(ctx);
}

static JSValueRef exclude(JSContextRef ctx, JSObjectRef /*function*/, JSObjectRef thisObject, size_t argumentCount, const JSValueRef arguments[], JSValueRef* /*exception*/)
{
    if (!JSValueIsObjectOfClass(ctx, thisObject, ProfileClass()))
        return JSValueMakeUndefined(ctx);

    if (argumentCount < 1)
        return JSValueMakeUndefined(ctx);

    if (!JSValueIsObjectOfClass(ctx, arguments[0], ProfileNodeClass()))
        return JSValueMakeUndefined(ctx);

    Profile* profile = static_cast<Profile*>(JSObjectGetPrivate(thisObject));
    profile->exclude(static_cast<ProfileNode*>(JSObjectGetPrivate(const_cast<JSObjectRef>(arguments[0]))));

    return JSValueMakeUndefined(ctx);
}

static JSValueRef restoreAll(JSContextRef ctx, JSObjectRef /*function*/, JSObjectRef thisObject, size_t /*argumentCount*/, const JSValueRef[] /*arguments*/, JSValueRef* /*exception*/)
{
    if (!JSValueIsObjectOfClass(ctx, thisObject, ProfileClass()))
        return JSValueMakeUndefined(ctx);

    Profile* profile = static_cast<Profile*>(JSObjectGetPrivate(thisObject));
    profile->restoreAll();

    return JSValueMakeUndefined(ctx);
}

static void finalize(JSObjectRef object)
{
    Profile* profile = static_cast<Profile*>(JSObjectGetPrivate(object));
    profileCache().remove(profile);
    profile->deref();
}

JSClassRef ProfileClass()
{
    static JSStaticValue staticValues[] = {
        { "title", getTitleCallback, 0, kJSPropertyAttributeNone },
        { "head", getHeadCallback, 0, kJSPropertyAttributeNone },
        { "uid", getUniqueIdCallback, 0, kJSPropertyAttributeNone },
        { 0, 0, 0, 0 }
    };

    static JSStaticFunction staticFunctions[] = {
        { "focus", focus, kJSPropertyAttributeNone },
        { "exclude", exclude, kJSPropertyAttributeNone },
        { "restoreAll", restoreAll, kJSPropertyAttributeNone },
        { 0, 0, 0 }
    };

    static JSClassDefinition classDefinition = {
        0, kJSClassAttributeNone, "Profile", 0, staticValues, staticFunctions,
        0, finalize, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    static JSClassRef profileClass = JSClassCreate(&classDefinition);
    return profileClass;
}

JSValue toJS(ExecState* exec, Profile* profile)
{
    if (!profile)
        return jsNull();

    JSObject* profileWrapper = profileCache().get(profile);
    if (profileWrapper)
        return profileWrapper;

    profile->ref();
    profileWrapper = toJS(JSObjectMake(toRef(exec), ProfileClass(), static_cast<void*>(profile)));
    profileCache().set(profile, profileWrapper);
    return profileWrapper;
}

} // namespace WebCore

#endif // ENABLE(JAVASCRIPT_DEBUGGER)
