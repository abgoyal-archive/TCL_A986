

#include "config.h"
#include "WebKitDLL.h"
#include "WebIconFetcher.h"

#include "MemoryStream.h"
#include <WebCore/COMPtr.h>
#include <WebCore/SharedBuffer.h>
#include <wtf/PassRefPtr.h>

using namespace WebCore;

class WebIconFetcherClient : public IconFetcherClient {
public:
    WebIconFetcherClient(IWebIconFetcherDelegate* delegate)
        : m_fetcher(0)
        , m_delegate(delegate)
    {
    }

    virtual void finishedFetchingIcon(PassRefPtr<SharedBuffer> iconData)
    {
        COMPtr<MemoryStream> memoryStream;
        if (iconData)
            memoryStream = MemoryStream::createInstance(iconData);

        m_delegate->finishedLoadingIcon(m_fetcher, memoryStream.get());

        delete this;
    }

   void setFetcher(WebIconFetcher *fetcher) { m_fetcher = fetcher; }
    
private:
    WebIconFetcher* m_fetcher;
    COMPtr<IWebIconFetcherDelegate> m_delegate;
};

// WebIconFetcher -------------------------------------------------------------------

WebIconFetcher* WebIconFetcher::fetchApplicationIcon(Frame* frame, IWebIconFetcherDelegate* delegate)
{
    WebIconFetcherClient* client = new WebIconFetcherClient(delegate);
    
    RefPtr<IconFetcher> fetcher = IconFetcher::create(frame, client);
    
    if (!fetcher)
        return 0;
    
    COMPtr<WebIconFetcher> iconFetcher = new WebIconFetcher(fetcher.release());
    client->setFetcher(iconFetcher.get());

    return iconFetcher.releaseRef();
}

WebIconFetcher::WebIconFetcher(PassRefPtr<IconFetcher> iconFetcher)
    : m_refCount(0)
    , m_iconFetcher(iconFetcher)
{
    gClassCount++;
}

WebIconFetcher::~WebIconFetcher()
{
    gClassCount--;
}

// IUnknown -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebIconFetcher::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<IUnknown*>(this);
    else if (IsEqualGUID(riid, IID_IWebIconFetcher))
        *ppvObject = static_cast<IWebIconFetcher*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE WebIconFetcher::AddRef()
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE WebIconFetcher::Release()
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete this;

    return newRef;
}

// IUnknown -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebIconFetcher::cancel()
{
    m_iconFetcher->cancel();

    return S_OK;
}
