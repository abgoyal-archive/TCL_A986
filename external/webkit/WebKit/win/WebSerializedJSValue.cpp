

#include "config.h"
#include "WebKitDLL.h"
#include "WebSerializedJSValue.h"

#include <WebCore/SerializedScriptValue.h>

using namespace WebCore;

WebSerializedJSValue::WebSerializedJSValue()
    : m_refCount(0)
{
    ++gClassCount;
    gClassNameCount.add("WebSerializedJSValue");
}

WebSerializedJSValue::~WebSerializedJSValue()
{
    --gClassCount;
    gClassNameCount.remove("WebSerializedJSValue");
}

COMPtr<WebSerializedJSValue> WebSerializedJSValue::createInstance()
{
    return new WebSerializedJSValue();
}

ULONG WebSerializedJSValue::AddRef()
{
    return ++m_refCount;
}

ULONG WebSerializedJSValue::Release()
{
    ULONG newRefCount = --m_refCount;
    if (!newRefCount)
        delete this;
    return newRefCount;
}

HRESULT WebSerializedJSValue::QueryInterface(REFIID riid, void** ppvObject)
{
    if (!ppvObject)
        return E_POINTER;
    *ppvObject = 0;

    if (IsEqualIID(riid, __uuidof(WebSerializedJSValue)))
        *ppvObject = this;
    else if (IsEqualIID(riid, __uuidof(IWebSerializedJSValue)))
        *ppvObject = static_cast<IWebSerializedJSValue*>(this);
    else if (IsEqualIID(riid, IID_IUnknown))
        *ppvObject = static_cast<IUnknown*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

HRESULT WebSerializedJSValue::serialize(JSContextRef sourceContext, JSValueRef value, JSValueRef* exception)
{
    ASSERT_ARG(value, value);
    ASSERT_ARG(sourceContext, sourceContext);

    if (!value || !sourceContext)
        return E_POINTER;

    m_value = SerializedScriptValue::create(sourceContext, value, exception);
        
    return S_OK;
}

HRESULT WebSerializedJSValue::deserialize(JSContextRef destinationContext, JSValueRef* outValue)
{
    if (!outValue)
        return E_POINTER;

    if (!m_value)
        *outValue = 0;
    else
        *outValue = m_value->deserialize(destinationContext, 0);

    return S_OK;
}
