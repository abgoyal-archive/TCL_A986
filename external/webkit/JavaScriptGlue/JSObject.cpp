

#include "config.h"
#include "JSObject.h"

#include "UserObjectImp.h"

JSUserObject::JSUserObject(JSObjectCallBacksPtr callBacks, JSObjectMarkProcPtr markProc, void *data, int dataType)
    : JSBase(kJSObjectTypeID), fCallBacks(*callBacks), fMarkProc(markProc), fData(data), fDataType(dataType)
{
}

JSUserObject::~JSUserObject()
{
    if (fCallBacks.dispose)
    {
        fCallBacks.dispose(fData);
    }
}

CFArrayRef JSUserObject::CopyPropertyNames(void)
{
    CFArrayRef result = 0;
    if (fCallBacks.copyPropertyNames)
    {
        result = fCallBacks.copyPropertyNames(fData);
    }
    return result;
}

JSUserObject* JSUserObject::CopyProperty(CFStringRef propertyName)
{
    JSUserObject* result = 0;
    if (fCallBacks.copyProperty)
    {
        result = (JSUserObject*)fCallBacks.copyProperty(fData, propertyName);
    }
    return result;
}

void JSUserObject::SetProperty(CFStringRef propertyName, JSUserObject* value)
{
    if (fCallBacks.setProperty)
    {
        fCallBacks.setProperty(fData, propertyName, (JSObjectRef)value);
    }

}

static JSValue JSC_HOST_CALL nativeCallFunction(ExecState* exec, JSObject* functionObject, JSValue  thisValue, const ArgList& args);
static JSValue  nativeCallFunction(ExecState* exec, JSObject* functionObject, JSValue  thisValue, const ArgList& args)
{
    return static_cast<UserObjectImp*>(functionObject)->callAsFunction(exec, asObject(thisValue), args);
}

CallType JSUserObject::getCallData(CallData& callData)
{
    if (!fCallBacks.callFunction)
        return CallTypeNone;

    callData.native.function = nativeCallFunction;
    return CallTypeHost;
}

JSUserObject* JSUserObject::CallFunction(JSUserObject* thisObj, CFArrayRef args)
{
    JSUserObject* result = 0;
    if (fCallBacks.callFunction)
    {
        result = (JSUserObject*)fCallBacks.callFunction(fData, (JSObjectRef)thisObj, args);
    }
    return result;

}

CFTypeRef JSUserObject::CopyCFValue() const
{
    CFTypeRef result = 0;
    if (fCallBacks.copyCFValue)
    {
        result = (JSUserObject*)fCallBacks.copyCFValue(fData);
    }
    return result;
}

UInt8 JSUserObject::Equal(JSBase* other)
{
    UInt8 result = false;
    JSUserObject* obj = (JSUserObject*)other;
    if (obj->GetTypeID() == kJSObjectTypeID)
    {
        if (fCallBacks.equal)
        {
            result = fCallBacks.equal(GetData(), obj->GetData());
        }
        else
        {
            CFTypeRef cf1 = CopyCFValue();
            CFTypeRef cf2 = obj->CopyCFValue();
            if (cf1 && cf2)
            {
                result = CFEqual(cf1, cf2);
            }
            ReleaseCFType(cf2);
            ReleaseCFType(cf1);
        }
    }
    return result;
}

void JSUserObject::Mark()
{
    if (fMarkProc)
    {
        fMarkProc(fData);
    }
}

void *JSUserObject::GetData()
{
    return fData;
}


