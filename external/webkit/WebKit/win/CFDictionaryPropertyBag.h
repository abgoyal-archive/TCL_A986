

#ifndef CFDictionaryPropertyBag_h
#define CFDictionaryPropertyBag_h

#include <WebCore/COMPtr.h>
#include <wtf/RetainPtr.h>

typedef struct __CFDictionary* CFMutableDictionaryRef;

class CFDictionaryPropertyBag : public IPropertyBag {
public:
    static COMPtr<CFDictionaryPropertyBag> createInstance();

    // IUnknown
    virtual ULONG STDMETHODCALLTYPE AddRef();
    virtual ULONG STDMETHODCALLTYPE Release();

    void setDictionary(CFMutableDictionaryRef dictionary);
    CFMutableDictionaryRef dictionary() const;

private:
    CFDictionaryPropertyBag();
    ~CFDictionaryPropertyBag();

    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID, void** ppvObject);

    // IPropertyBag
    virtual HRESULT STDMETHODCALLTYPE Read(LPCOLESTR pszPropName, VARIANT*, IErrorLog*);
    virtual HRESULT STDMETHODCALLTYPE Write(LPCOLESTR pszPropName, VARIANT*);

    RetainPtr<CFMutableDictionaryRef> m_dictionary;
    ULONG m_refCount;
};

#endif // CFDictionaryPropertyBag_h
