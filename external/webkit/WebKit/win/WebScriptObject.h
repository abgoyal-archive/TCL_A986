

#ifndef WebScriptObject_H
#define WebScriptObject_H

#include "WebKit.h"

class WebScriptObject : public IWebScriptObject
{
public:
    WebScriptObject();
    virtual ~WebScriptObject();

    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebScriptObject
    virtual HRESULT STDMETHODCALLTYPE throwException( 
        /* [in] */ BSTR exceptionMessage,
        /* [retval][out] */ BOOL *result);
    
    virtual HRESULT STDMETHODCALLTYPE callWebScriptMethod( 
        /* [in] */ BSTR name,
        /* [size_is][in] */ const VARIANT args[  ],
        /* [in] */ int cArgs,
        /* [retval][out] */ VARIANT *result);
    
    virtual HRESULT STDMETHODCALLTYPE evaluateWebScript( 
        /* [in] */ BSTR script,
        /* [retval][out] */ VARIANT *result);
    
    virtual HRESULT STDMETHODCALLTYPE removeWebScriptKey( 
        /* [in] */ BSTR name);
    
    virtual HRESULT STDMETHODCALLTYPE stringRepresentation( 
        /* [retval][out] */ BSTR* stringRepresentation);
    
    virtual HRESULT STDMETHODCALLTYPE webScriptValueAtIndex( 
        /* [in] */ unsigned int index,
        /* [retval][out] */ VARIANT *result);
    
    virtual HRESULT STDMETHODCALLTYPE setWebScriptValueAtIndex( 
        /* [in] */ unsigned int index,
        /* [in] */ VARIANT val);
    
    virtual HRESULT STDMETHODCALLTYPE setException( 
        /* [in] */ BSTR description);

protected:
    ULONG m_refCount;
};

#endif
