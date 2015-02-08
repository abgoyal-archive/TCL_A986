

#ifndef WebKitStatistics_h
#define WebkitStatistics_h

#include "WebKit.h"

class WebKitStatistics : public IWebKitStatistics {
public:
    static WebKitStatistics* createInstance();
protected:
    WebKitStatistics();
    ~WebKitStatistics();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebKitStatistics
    virtual HRESULT STDMETHODCALLTYPE webViewCount( 
        /* [retval][out] */ int *count);
    
    virtual HRESULT STDMETHODCALLTYPE frameCount( 
        /* [retval][out] */ int *count);
    
    virtual HRESULT STDMETHODCALLTYPE dataSourceCount( 
        /* [retval][out] */ int *count);
    
    virtual HRESULT STDMETHODCALLTYPE viewCount( 
        /* [retval][out] */ int *count);
    
    virtual HRESULT STDMETHODCALLTYPE HTMLRepresentationCount( 
        /* [retval][out] */ int *count);

    virtual HRESULT STDMETHODCALLTYPE comClassCount( 
        /* [retval][out] */ int *classCount);

    virtual HRESULT STDMETHODCALLTYPE comClassNameCounts( 
        /* [retval][out] */ BSTR *output);

protected:
    ULONG m_refCount;
};

#endif
