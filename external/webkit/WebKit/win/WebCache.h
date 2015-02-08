

#ifndef WebCache_H
#define WebCache_H

#include "WebKit.h"

class WebCache : public IWebCache
{
public:
    static WebCache* createInstance();
protected:
    WebCache();
    ~WebCache();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebCache
    virtual HRESULT STDMETHODCALLTYPE statistics( 
        /* [in][out] */ int* count,
        /* [retval][out] */ IPropertyBag **s);
    
    virtual HRESULT STDMETHODCALLTYPE empty( void);
    
    virtual HRESULT STDMETHODCALLTYPE setDisabled( 
        /* [in] */ BOOL disabled);

    virtual HRESULT STDMETHODCALLTYPE disabled(
        /* [out][retval] */ BOOL*);

protected:
    ULONG m_refCount;
};

#endif
