

#ifndef WebNavigationData_h
#define WebNavigationData_h

#include "WebKit.h"

#include "COMPtr.h"
#include <WebCore/BString.h>

class WebNavigationData : public IWebNavigationData {
public:
    static WebNavigationData* createInstance(const WebCore::String& url, const WebCore::String& title, IWebURLRequest*, IWebURLResponse*, bool hasSubstituteData, const WebCore::String& clientRedirectSource);
private:
    WebNavigationData(const WebCore::String& url, const WebCore::String& title, IWebURLRequest*, IWebURLResponse*, bool hasSubstituteData, const WebCore::String& clientRedirectSource);
    ~WebNavigationData();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebNavigationData
    virtual HRESULT STDMETHODCALLTYPE url(
        /* [retval][out] */ BSTR*);

    virtual HRESULT STDMETHODCALLTYPE title(
        /* [retval][out] */ BSTR*);

    virtual HRESULT STDMETHODCALLTYPE originalRequest(
        /* [retval][out] */ IWebURLRequest**);

    virtual HRESULT STDMETHODCALLTYPE response(
        /* [retval][out] */ IWebURLResponse**);

    virtual HRESULT STDMETHODCALLTYPE hasSubstituteData(
        /* [retval][out] */ BOOL*);

    virtual HRESULT STDMETHODCALLTYPE clientRedirectSource(
        /* [retval][out] */ BSTR*);

private:
    ULONG m_refCount;
    WebCore::BString m_url;
    WebCore::BString m_title;
    COMPtr<IWebURLRequest> m_request;
    COMPtr<IWebURLResponse> m_response;
    bool m_hasSubstituteData;
    WebCore::BString m_clientRedirectSource;

};

#endif // WebNavigationData_h
