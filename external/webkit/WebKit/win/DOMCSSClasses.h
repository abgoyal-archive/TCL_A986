

#ifndef DOMCSSClasses_H
#define DOMCSSClasses_H

#include "WebKit.h"
#include "DOMCoreClasses.h"

#include <WebCore/CSSStyleDeclaration.h>

class DOMCSSStyleDeclaration : public DOMObject, public IDOMCSSStyleDeclaration
{
protected:
    DOMCSSStyleDeclaration(WebCore::CSSStyleDeclaration* d);
    ~DOMCSSStyleDeclaration();

public:
    static IDOMCSSStyleDeclaration* createInstance(WebCore::CSSStyleDeclaration* d);

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void) { return DOMObject::AddRef(); }
    virtual ULONG STDMETHODCALLTYPE Release(void) { return DOMObject::Release(); }

    // IWebScriptObject
    virtual HRESULT STDMETHODCALLTYPE throwException( 
        /* [in] */ BSTR exceptionMessage,
        /* [retval][out] */ BOOL *result) { return DOMObject::throwException(exceptionMessage, result); }
    
    virtual HRESULT STDMETHODCALLTYPE callWebScriptMethod( 
        /* [in] */ BSTR name,
        /* [size_is][in] */ const VARIANT args[  ],
        /* [in] */ int cArgs,
        /* [retval][out] */ VARIANT *result) { return DOMObject::callWebScriptMethod(name, args, cArgs, result); }
    
    virtual HRESULT STDMETHODCALLTYPE evaluateWebScript( 
        /* [in] */ BSTR script,
        /* [retval][out] */ VARIANT *result) { return DOMObject::evaluateWebScript(script, result); }
    
    virtual HRESULT STDMETHODCALLTYPE removeWebScriptKey( 
        /* [in] */ BSTR name) { return DOMObject::removeWebScriptKey(name); }
    
    virtual HRESULT STDMETHODCALLTYPE stringRepresentation( 
        /* [retval][out] */ BSTR* stringRepresentation) { return DOMObject::stringRepresentation(stringRepresentation); }
    
    virtual HRESULT STDMETHODCALLTYPE webScriptValueAtIndex( 
        /* [in] */ unsigned int index,
        /* [retval][out] */ VARIANT *result) { return DOMObject::webScriptValueAtIndex(index, result); }
    
    virtual HRESULT STDMETHODCALLTYPE setWebScriptValueAtIndex( 
        /* [in] */ unsigned int index,
        /* [in] */ VARIANT val) { return DOMObject::setWebScriptValueAtIndex(index, val); }
    
    virtual HRESULT STDMETHODCALLTYPE setException( 
        /* [in] */ BSTR description) { return DOMObject::setException(description); }

    // IDOMCSSStyleDeclaration
    virtual HRESULT STDMETHODCALLTYPE cssText( 
        /* [retval][out] */ BSTR *result);
    
    virtual HRESULT STDMETHODCALLTYPE setCssText( 
        /* [in] */ BSTR cssText);
    
    virtual HRESULT STDMETHODCALLTYPE getPropertyValue( 
        /* [in] */ BSTR propertyName,
        /* [retval][out] */ BSTR *result);
    
    virtual HRESULT STDMETHODCALLTYPE getPropertyCSSValue( 
        /* [in] */ BSTR propertyName,
        /* [retval][out] */ IDOMCSSValue **result);
    
    virtual HRESULT STDMETHODCALLTYPE removeProperty( 
        /* [in] */ BSTR propertyName,
        /* [retval][out] */ BSTR *result);
    
    virtual HRESULT STDMETHODCALLTYPE getPropertyPriority( 
        /* [in] */ BSTR propertyName,
        /* [retval][out] */ BSTR *result);
    
    virtual HRESULT STDMETHODCALLTYPE setProperty( 
        /* [in] */ BSTR propertyName,
        /* [in] */ BSTR value,
        /* [in] */ BSTR priority);
    
    virtual HRESULT STDMETHODCALLTYPE length( 
        /* [retval][out] */ UINT *result);
    
    virtual HRESULT STDMETHODCALLTYPE item( 
        /* [in] */ UINT index,
        /* [retval][out] */ BSTR *result);
    
    virtual HRESULT STDMETHODCALLTYPE parentRule( 
        /* [retval][out] */ IDOMCSSRule **result);

protected:
    ULONG m_refCount;
    WebCore::CSSStyleDeclaration* m_style;
};

#endif
