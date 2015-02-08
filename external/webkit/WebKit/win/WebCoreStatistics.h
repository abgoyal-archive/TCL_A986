

#ifndef WebCoreStatistics_h
#define WebCoreStatistics_h

#include "WebKit.h"

class WebCoreStatistics : public IWebCoreStatistics {
public:
    static WebCoreStatistics* createInstance();
protected:
    WebCoreStatistics();
    ~WebCoreStatistics();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebCoreStatistics
    virtual HRESULT STDMETHODCALLTYPE javaScriptObjectsCount( 
        /* [retval][out] */ UINT *count);
    virtual HRESULT STDMETHODCALLTYPE javaScriptGlobalObjectsCount( 
        /* [retval][out] */ UINT *count);
    virtual HRESULT STDMETHODCALLTYPE javaScriptProtectedObjectsCount( 
        /* [retval][out] */ UINT *count);
    virtual HRESULT STDMETHODCALLTYPE javaScriptProtectedGlobalObjectsCount( 
        /* [retval][out] */ UINT *count);
    virtual HRESULT STDMETHODCALLTYPE javaScriptProtectedObjectTypeCounts( 
        /* [retval][out] */ IPropertyBag2** typeNamesAndCounts);
    virtual HRESULT STDMETHODCALLTYPE iconPageURLMappingCount( 
        /* [retval][out] */ UINT *count);
    virtual HRESULT STDMETHODCALLTYPE iconRetainedPageURLCount( 
        /* [retval][out] */ UINT *count);
    virtual HRESULT STDMETHODCALLTYPE iconRecordCount( 
        /* [retval][out] */ UINT *count);
    virtual HRESULT STDMETHODCALLTYPE iconsWithDataCount( 
        /* [retval][out] */ UINT *count);
    virtual HRESULT STDMETHODCALLTYPE cachedFontDataCount( 
        /* [retval][out] */ UINT *count);
    virtual HRESULT STDMETHODCALLTYPE cachedFontDataInactiveCount( 
        /* [retval][out] */ UINT *count);
    virtual HRESULT STDMETHODCALLTYPE purgeInactiveFontData(void);
    virtual HRESULT STDMETHODCALLTYPE glyphPageCount( 
        /* [retval][out] */ UINT *count);

protected:
    ULONG m_refCount;
};

#endif
