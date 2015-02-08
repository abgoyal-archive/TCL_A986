

#include "config.h"
#include "JavaScriptGlue.h"
#include "JSUtils.h"
#include "JSBase.h"
#include "JSObject.h"
#include "JSRun.h"
#include <JavaScriptCore/Completion.h>
#include <JavaScriptCore/InitializeThreading.h>

static CFTypeRef sJSCFNullRef = 0;

static void CFJSObjectDispose(void *data);
static JSObjectRef CFJSObjectCopyProperty(void *data, CFStringRef propertyName);
static void CFJSObjectSetProperty(void *data, CFStringRef propertyName, JSObjectRef jsValue);
static CFTypeRef CFJSObjectCopyCFValue(void *data);
static UInt8 CFJSObjectEqual(void *data1, void *data2);
static CFArrayRef CFJSObjectCopyPropertyNames(void *data);

void *JSCFRetain(CFAllocatorRef allocator, const void *value);
void JSCFRelease(CFAllocatorRef allocator, const void *value);


void JSSetCFNull(CFTypeRef nullRef)
{
    ReleaseCFType(sJSCFNullRef);
    sJSCFNullRef = RetainCFType(nullRef);
}

CFTypeRef JSGetCFNull(void)
{
    return sJSCFNullRef;
}

JSTypeRef JSRetain(JSTypeRef ref)
{
    if (ref)
    {
        JSBase* ptr = (JSBase*)ref;
        ptr->Retain();
    }
    return ref;
}

void JSRelease(JSTypeRef ref)
{
    if (ref)
    {
        JSBase* ptr = (JSBase*)ref;
        ptr->Release();
    }
}

CFStringRef JSCopyDescription(JSTypeRef ref)
{
    CFStringRef result = 0;
    if (ref)
    {
        JSBase* ptr = (JSBase*)ref;
        ptr->CopyDescription();
    }
    return result;
}

UInt8 JSEqual(JSTypeRef ref1, JSTypeRef ref2)
{
    UInt8 result = false;
    if (ref1 && ref2)
    {
        JSBase* ptr = (JSBase*)ref1;
        result = ptr->Equal((JSBase*)ref2);
    }
    return result;
}


JSTypeID JSGetTypeID(JSTypeRef ref)
{
    JSTypeID result = kJSInvalidTypeID;
    if (ref)
    {
        JSBase* ptr = (JSBase*)ref;
        result = ptr->GetTypeID();
    }
    return result;
}


CFIndex JSGetRetainCount(JSTypeRef ref)
{
    CFIndex result = -1;
    if (ref)
    {
        JSBase* ptr = (JSBase*)ref;
        result = ptr->RetainCount();
    }
    return result;
}



JSObjectRef JSObjectCreate(void *data, JSObjectCallBacksPtr callBacks)
{
    JSObjectRef result = JSObjectCreateInternal(data, callBacks, 0, kJSUserObjectDataTypeUnknown);
    return result;
}

JSObjectRef JSObjectCreateInternal(void *data, JSObjectCallBacksPtr callBacks, JSObjectMarkProcPtr markProc, int type)
{
    JSObjectRef result = 0;
    JSUserObject* ptr = new JSUserObject(callBacks, markProc, data, type);
    result = (JSObjectRef)ptr;
    return result;
}

CFTypeRef JSObjectCopyCFValue(JSObjectRef ref)
{
    CFTypeRef result = 0;
    JSUserObject* ptr = (JSUserObject*)ref;
    if (ptr && (ptr->GetTypeID() == kJSObjectTypeID))
    {
        result = ptr->CopyCFValue();
    }
    return result;
}

void *JSObjectGetData(JSObjectRef ref)
{
    void *result = 0;
    JSUserObject* ptr = (JSUserObject*)ref;
    if (ptr && (ptr->GetTypeID() == kJSObjectTypeID))
    {
        result = ptr->GetData();
    }
    return result;
}


JSObjectRef JSObjectCopyProperty(JSObjectRef ref, CFStringRef propertyName)
{
    JSObjectRef result = 0;
    JSUserObject* ptr = (JSUserObject*)ref;
    if (ptr && (ptr->GetTypeID() == kJSObjectTypeID))
    {
        result = (JSObjectRef)ptr->CopyProperty(propertyName);
    }
    return result;
}


void JSObjectSetProperty(JSObjectRef ref, CFStringRef propertyName, JSObjectRef value)
{
    JSUserObject* ptr = (JSUserObject*)ref;
    if (ptr && (ptr->GetTypeID() == kJSObjectTypeID))
    {
        ptr->SetProperty(propertyName, (JSUserObject*)value);
    }
}


JSObjectRef JSObjectCallFunction(JSObjectRef ref, JSObjectRef thisObj, CFArrayRef args)
{
    JSObjectRef result = 0;
    JSUserObject* ptr = (JSUserObject*)ref;
    if (ptr && (ptr->GetTypeID() == kJSObjectTypeID))
    {
        result = (JSObjectRef)ptr->CallFunction((JSUserObject*)thisObj, args);
    }
    return result;
}


JSRunRef JSRunCreate(CFStringRef jsSource, JSFlags inFlags)
{
    initializeThreading();

    JSRunRef result = 0;
    if (jsSource)
    {
        JSGlueAPIEntry entry;
        result = (JSRunRef) new JSRun(jsSource, inFlags);
    }
    return result;
}

CFStringRef JSRunCopySource(JSRunRef ref)
{
    CFStringRef result = 0;
    JSRun* ptr = (JSRun*)ref;
    if (ptr)
    {
        result = UStringToCFString(ptr->GetSource());
    }
    return result;
}


JSObjectRef JSRunCopyGlobalObject(JSRunRef ref)
{
    JSObjectRef result = 0;
    JSRun* ptr = (JSRun*)ref;
    if (ptr)
    {
        JSGlobalObject* globalObject = ptr->GlobalObject();
        result = (JSObjectRef)KJSValueToJSObject(globalObject, globalObject->globalExec());
    }
    return result;
}

JSObjectRef JSRunEvaluate(JSRunRef ref)
{
    JSObjectRef result = 0;
    JSRun* ptr = (JSRun*)ref;
    if (ptr)
    {
        JSGlueAPIEntry entry;
        Completion completion = ptr->Evaluate();
        if (completion.isValueCompletion())
        {
            result = (JSObjectRef)KJSValueToJSObject(completion.value(), ptr->GlobalObject()->globalExec());
        }

        if (completion.complType() == Throw)
        {
            JSFlags flags = ptr->Flags();
            if (flags & kJSFlagDebug)
            {
                CFTypeRef error = JSObjectCopyCFValue(result);
                if (error)
                {
                    CFShow(error);
                    CFRelease(error);
                }
            }
        }
    }
    return result;
}

bool JSRunCheckSyntax(JSRunRef ref)
{
    bool result = false;
    JSRun* ptr = (JSRun*)ref;
    if (ptr)
    {
            JSGlueAPIEntry entry;
            result = ptr->CheckSyntax();
    }
    return result;
}

void JSCollect()
{
    initializeThreading();

    JSGlueAPIEntry entry;
    Heap* heap = getThreadGlobalExecState()->heap();
    if (!heap->isBusy())
        heap->collectAllGarbage();
}

void JSTypeGetCFArrayCallBacks(CFArrayCallBacks* outCallBacks)
{
    if (outCallBacks)
    {
        outCallBacks->version = 1;
        outCallBacks->retain = (CFArrayRetainCallBack)JSCFRetain;
        outCallBacks->release = (CFArrayReleaseCallBack)JSCFRelease;
        outCallBacks->copyDescription = (CFArrayCopyDescriptionCallBack)JSCopyDescription;
        outCallBacks->equal = (CFArrayEqualCallBack)JSEqual;
    }
}


void *JSCFRetain(CFAllocatorRef allocator, const void *value)
{
    JSRetain((JSTypeRef)value);
    return (void*)value;
}

void JSCFRelease(CFAllocatorRef allocator, const void *value)
{
    JSRelease((JSTypeRef)value);
}


JSObjectRef JSObjectCreateWithCFType(CFTypeRef inRef)
{
    JSObjectCallBacks callBacks;
    JSObjectRef cfJSObject = nil;
    if (inRef)
    {
        callBacks.dispose = CFJSObjectDispose;
        callBacks.equal = CFJSObjectEqual;
        callBacks.copyCFValue = CFJSObjectCopyCFValue;
        callBacks.copyProperty = CFJSObjectCopyProperty;
        callBacks.setProperty = CFJSObjectSetProperty;
        callBacks.callFunction = 0;
        callBacks.copyPropertyNames = CFJSObjectCopyPropertyNames;
        cfJSObject = JSObjectCreateInternal((void*)CFRetain(inRef), &callBacks, 0, kJSUserObjectDataTypeCFType );
    }
    return cfJSObject;
}

void CFJSObjectDispose(void *data)
{
    if (data)
    {
        CFRelease((JSTypeRef)data);
    }
}

CFArrayRef JSObjectCopyPropertyNames(JSObjectRef ref)
{
    CFArrayRef result = 0;
    JSUserObject* ptr = (JSUserObject*)ref;
    if (ptr && (ptr->GetTypeID() == kJSObjectTypeID))
    {
        result = ptr->CopyPropertyNames();
    }
    return result;
}
JSObjectRef CFJSObjectCopyProperty(void *data, CFStringRef propertyName)
{
    JSObjectRef result = 0;
    if (data && propertyName)
    {
        CFTypeRef cfResult = 0;
        if (CFGetTypeID(data) == CFDictionaryGetTypeID())
        {
            if (CFStringCompare(propertyName, CFSTR("length"), 0) == kCFCompareEqualTo)
            {
                int len = CFDictionaryGetCount((CFDictionaryRef)data);
                cfResult = CFNumberCreate(0, kCFNumberIntType, &len);
            }
            else
            {
                cfResult = RetainCFType(CFDictionaryGetValue((CFDictionaryRef)data, propertyName));
            }
        }
        else if (CFGetTypeID(data) == CFArrayGetTypeID())
        {
            if (CFStringCompare(propertyName, CFSTR("length"), 0) == kCFCompareEqualTo)
            {
                int len = CFArrayGetCount((CFArrayRef)data);
                cfResult = CFNumberCreate(0, kCFNumberIntType, &len);
            }
            else
            {
                SInt32 index = CFStringGetIntValue(propertyName);
                CFIndex arrayCount = CFArrayGetCount((CFArrayRef)data);
                if (index >= 0 && index < arrayCount)
                {
                    cfResult = RetainCFType(CFArrayGetValueAtIndex((CFArrayRef)data, index));
                }
            }
        }
        else if (CFGetTypeID(data) == CFStringGetTypeID())
        {
            if (CFStringCompare(propertyName, CFSTR("length"), 0) == kCFCompareEqualTo)
            {
                int len = CFStringGetLength((CFStringRef)data);
                cfResult = CFNumberCreate(0, kCFNumberIntType, &len);
            }
        }
        if (cfResult)
        {
            result = JSObjectCreateWithCFType(cfResult);
            CFRelease(cfResult);
        }
    }
    return result;
}


void CFJSObjectSetProperty(void *data, CFStringRef propertyName, JSObjectRef jsValue)
{
    if (data && propertyName)
    {
        CFTypeRef cfValue = JSObjectCopyCFValue(jsValue);

        if (cfValue)
        {
            if (CFGetTypeID(data) == CFDictionaryGetTypeID())
            {
                CFDictionarySetValue((CFMutableDictionaryRef)data, propertyName, cfValue);
            }
            else if (CFGetTypeID(data) == CFArrayGetTypeID())
            {
                SInt32 index = CFStringGetIntValue(propertyName);
                CFIndex arrayCount = CFArrayGetCount((CFArrayRef)data);
                if (index >= 0)
                {
                    for (; arrayCount < index; arrayCount++)
                    {
                        CFArrayAppendValue((CFMutableArrayRef)data, GetCFNull());
                    }
                    CFArraySetValueAtIndex((CFMutableArrayRef)data, index, cfValue);
                }
            }
            CFRelease(cfValue);
        }
        else
        {
            if (CFGetTypeID(data) == CFDictionaryGetTypeID())
            {
                CFDictionaryRemoveValue((CFMutableDictionaryRef)data, propertyName);
            }
            else if (CFGetTypeID(data) == CFArrayGetTypeID())
            {
                SInt32 index = CFStringGetIntValue(propertyName);
                CFIndex arrayCount = CFArrayGetCount((CFArrayRef)data);
                if (index >= 0)
                {
                    for (; arrayCount < index; arrayCount++)
                    {
                        CFArrayAppendValue((CFMutableArrayRef)data, GetCFNull());
                    }
                    CFArraySetValueAtIndex((CFMutableArrayRef)data, index, GetCFNull());
                }
            }
        }
    }
}


CFTypeRef CFJSObjectCopyCFValue(void *data)
{
    CFTypeRef result = 0;
    if (data)
    {
        result = (CFTypeRef)CFRetain(data);
    }
    return result;
}

UInt8 CFJSObjectEqual(void *data1, void *data2)
{
    UInt8 result = false;
    if (data1 && data2)
    {
        CFEqual((CFTypeRef)data1, (CFTypeRef)data2);
    }
    return result;
}


CFArrayRef CFJSObjectCopyPropertyNames(void *data)
{
    CFMutableArrayRef result = 0;
    if (data)
    {
        CFTypeID cfType = CFGetTypeID(data);
        if (cfType == CFDictionaryGetTypeID())
        {
            CFIndex count = CFDictionaryGetCount((CFDictionaryRef)data);
            if (count)
            {
                CFTypeRef* keys = (CFTypeRef*)malloc(sizeof(CFTypeRef)*count);
                if (keys)
                {
                    int i;
                    CFDictionaryGetKeysAndValues((CFDictionaryRef)data, (const void **)keys, 0);
                    for (i = 0; i < count; i++)
                    {
                        CFStringRef key = (CFStringRef)keys[i];
                        if (CFGetTypeID(key) != CFStringGetTypeID()) continue;

                        if (!result) result = CFArrayCreateMutable(0, 0, &kCFTypeArrayCallBacks);
                        if (!result) continue;

                        CFArrayAppendValue(result, key);
                    }
                    free(keys);
                }
            }
        }
    }
    return result;
}




CFMutableArrayRef JSCreateCFArrayFromJSArray(CFArrayRef array)
{
    CFIndex count = array ? CFArrayGetCount(array) : 0;
    CFMutableArrayRef cfArray = CFArrayCreateMutable(0, 0, &kCFTypeArrayCallBacks);
    CFIndex i;

    for (i = 0; cfArray && i <  count; i++)
    {
        JSObjectRef jsValue = (JSObjectRef)CFArrayGetValueAtIndex(array, i);
        CFTypeRef cfvalue = JSObjectCopyCFValue(jsValue);
        if (cfvalue)
        {
            CFArrayAppendValue(cfArray, cfvalue);
            CFRelease(cfvalue);
        }
        else
        {
            CFArrayAppendValue(cfArray, GetCFNull());
        }
    }
    return cfArray;
}

CFMutableArrayRef JSCreateJSArrayFromCFArray(CFArrayRef array)
{
    initializeThreading();

    CFIndex count = array ? CFArrayGetCount(array) : 0;
    CFArrayCallBacks arrayCallbacks;
    CFMutableArrayRef jsArray;
    CFIndex i;

    JSTypeGetCFArrayCallBacks(&arrayCallbacks);
    jsArray = CFArrayCreateMutable(0, 0, &arrayCallbacks);

    for (i = 0; array && i <  count; i++)
    {
        CFTypeRef cfValue = (CFTypeRef)CFArrayGetValueAtIndex(array, i);
        JSObjectRef jsValue = JSObjectCreateWithCFType(cfValue);

        if (!jsValue) jsValue = JSObjectCreateWithCFType(GetCFNull());
        if (jsValue)
        {
            CFArrayAppendValue(jsArray, jsValue);
            JSRelease(jsValue);
        }
    }
    return jsArray;
}


void JSLockInterpreter()
{
    initializeThreading();
    JSLock::lock(LockForReal);
}


void JSUnlockInterpreter()
{
    JSLock::unlock(LockForReal);
}
