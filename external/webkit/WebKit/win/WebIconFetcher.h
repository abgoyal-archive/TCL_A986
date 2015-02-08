

#ifndef WebIconFetcher_h
#define WebIconFetcher_h

#include "WebKit.h"
#include <WebCore/IconFetcher.h>

class WebIconFetcher : public IWebIconFetcher {
public:
    static WebIconFetcher* fetchApplicationIcon(WebCore::Frame*, IWebIconFetcherDelegate*);

    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef();
    virtual ULONG STDMETHODCALLTYPE Release();

    // IWebIconFetcher
    virtual HRESULT STDMETHODCALLTYPE cancel();

private:
    WebIconFetcher(PassRefPtr<WebCore::IconFetcher>);
    ~WebIconFetcher();

    ULONG m_refCount;
    RefPtr<WebCore::IconFetcher> m_iconFetcher;
};

#endif // WebIconFetcher_h
