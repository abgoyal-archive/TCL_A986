

#include "config.h"
#include "JSValueWrapper.h"
#include "JSRun.h"
#include <JavaScriptCore/JSArray.h>
#include <JavaScriptCore/PropertyNameArray.h>
#include <pthread.h>

JSValueWrapper::JSValueWrapper(JSValue inValue)
    : fValue(inValue)
{
}

JSValueWrapper::~JSValueWrapper()
{
}

JSValue JSValueWrapper::GetValue()
{
    return fValue.get();
}

void JSValueWrapper::GetJSObectCallBacks(JSObjectCallBacks& callBacks)
{
    callBacks.dispose = (JSObjectDisposeProcPtr)JSValueWrapper::JSObjectDispose;
    callBacks.equal = (JSObjectEqualProcPtr)0;
    callBacks.copyPropertyNames = (JSObjectCopyPropertyNamesProcPtr)JSValueWrapper::JSObjectCopyPropertyNames;
    callBacks.copyCFValue = (JSObjectCopyCFValueProcPtr)JSValueWrapper::JSObjectCopyCFValue;
    callBacks.copyProperty = (JSObjectCopyPropertyProcPtr)JSValueWrapper::JSObjectCopyProperty;
    callBacks.setProperty = (JSObjectSetPropertyProcPtr)JSValueWrapper::JSObjectSetProperty;
    callBacks.callFunction = (JSObjectCallFunctionProcPtr)JSValueWrapper::JSObjectCallFunction;
}

void JSValueWrapper::JSObjectDispose(void *data)
{
    JSValueWrapper* ptr = (JSValueWrapper*)data;
    delete ptr;
}


CFArrayRef JSValueWrapper::JSObjectCopyPropertyNames(void *data)
{
    JSGlueAPIEntry entry;

    CFMutableArrayRef result = 0;
    JSValueWrapper* ptr = (JSValueWrapper*)data;
    if (ptr)
    {
        ExecState* exec = getThreadGlobalExecState();
        JSObject* object = ptr->GetValue().toObject(exec);
        PropertyNameArray propNames(exec);
        object->getPropertyNames(exec, propNames);
        PropertyNameArray::const_iterator iterator = propNames.begin();

        while (iterator != propNames.end()) {
            Identifier name = *iterator;
            CFStringRef nameStr = IdentifierToCFString(name);

            if (!result)
            {
                result = CFArrayCreateMutable(0, 0, &kCFTypeArrayCallBacks);
            }
            if (result && nameStr)
            {
                CFArrayAppendValue(result, nameStr);
            }
            ReleaseCFType(nameStr);
            iterator++;
        }

    }
    return result;
}


JSObjectRef JSValueWrapper::JSObjectCopyProperty(void *data, CFStringRef propertyName)
{
    JSGlueAPIEntry entry;

    JSObjectRef result = 0;
    JSValueWrapper* ptr = (JSValueWrapper*)data;
    if (ptr)
    {
        ExecState* exec = getThreadGlobalExecState();
        JSValue propValue = ptr->GetValue().toObject(exec)->get(exec, CFStringToIdentifier(propertyName, exec));
        JSValueWrapper* wrapperValue = new JSValueWrapper(propValue);

        JSObjectCallBacks callBacks;
        GetJSObectCallBacks(callBacks);
        result = JSObjectCreateInternal(wrapperValue, &callBacks, JSValueWrapper::JSObjectMark, kJSUserObjectDataTypeJSValueWrapper);

        if (!result)
        {
            delete wrapperValue;
        }
    }
    return result;
}

void JSValueWrapper::JSObjectSetProperty(void *data, CFStringRef propertyName, JSObjectRef jsValue)
{
    JSGlueAPIEntry entry;

    JSValueWrapper* ptr = (JSValueWrapper*)data;
    if (ptr)
    {
        ExecState* exec = getThreadGlobalExecState();
        JSValue value = JSObjectKJSValue((JSUserObject*)jsValue);
        JSObject *objValue = ptr->GetValue().toObject(exec);
        PutPropertySlot slot;
        objValue->put(exec, CFStringToIdentifier(propertyName, exec), value, slot);
    }
}

JSObjectRef JSValueWrapper::JSObjectCallFunction(void *data, JSObjectRef thisObj, CFArrayRef args)
{
    JSGlueAPIEntry entry;

    JSObjectRef result = 0;
    JSValueWrapper* ptr = (JSValueWrapper*)data;
    if (ptr)
    {
        ExecState* exec = getThreadGlobalExecState();

        JSValue value = JSObjectKJSValue((JSUserObject*)thisObj);
        JSObject* ksjThisObj = value.toObject(exec);
        JSObject* objValue = ptr->GetValue().toObject(exec);

        MarkedArgumentBuffer listArgs;
        CFIndex argCount = args ? CFArrayGetCount(args) : 0;
        for (CFIndex i = 0; i < argCount; i++)
        {
            JSObjectRef jsArg = (JSObjectRef)CFArrayGetValueAtIndex(args, i);
            JSValue kgsArg = JSObjectKJSValue((JSUserObject*)jsArg);
            listArgs.append(kgsArg);
        }

        CallData callData;
        CallType callType = objValue->getCallData(callData);
        if (callType == CallTypeNone)
            return 0;
        JSValue  resultValue = call(exec, objValue, callType, callData, ksjThisObj, listArgs);
        JSValueWrapper* wrapperValue = new JSValueWrapper(resultValue);
        JSObjectCallBacks callBacks;
        GetJSObectCallBacks(callBacks);
        result = JSObjectCreate(wrapperValue, &callBacks);
        if (!result)
        {
            delete wrapperValue;
        }
    }
    return result;
}

CFTypeRef JSValueWrapper::JSObjectCopyCFValue(void *data)
{
    JSGlueAPIEntry entry;

    CFTypeRef result = 0;
    JSValueWrapper* ptr = (JSValueWrapper*)data;
    if (ptr)
    {
        result = KJSValueToCFType(ptr->GetValue(), getThreadGlobalExecState());
    }
    return result;
}

void JSValueWrapper::JSObjectMark(void *data)
{
    JSValueWrapper* ptr = (JSValueWrapper*)data;
    if (ptr)
    {
        // This results in recursive marking but will be otherwise safe and correct.
        // We claim the array vptr is 0 because we don't have access to it here, and
        // claiming 0 is functionally harmless -- it merely means that we can't
        // devirtualise marking of arrays when recursing from this point.
        MarkStack markStack(0);
        markStack.append(ptr->fValue.get());
        markStack.drain();
    }
}
