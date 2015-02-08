

#ifndef WebSerializedJSValue_h
#define WebSerializedJSValue_h

#include <WebCore/COMPtr.h>

typedef const struct OpaqueJSContext* JSContextRef;
typedef const struct OpaqueJSValue* JSValueRef;

namespace WebCore {
    class SerializedScriptValue;
}

class WebSerializedJSValue : public Noncopyable, public IWebSerializedJSValue {
public:
    static COMPtr<WebSerializedJSValue> createInstance();

    virtual ULONG STDMETHODCALLTYPE AddRef();
    virtual ULONG STDMETHODCALLTYPE Release();

    virtual HRESULT STDMETHODCALLTYPE serialize(JSContextRef, JSValueRef value, JSValueRef* exception);
    virtual HRESULT STDMETHODCALLTYPE deserialize(JSContextRef, JSValueRef* result);

private:
    WebSerializedJSValue();
    ~WebSerializedJSValue();

    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID, void** ppvObject);

    ULONG m_refCount;
    RefPtr<WebCore::SerializedScriptValue> m_value;
};

#endif // WebSerializedJSValue_h
