

#ifndef WebArchive_h
#define WebArchive_h

#include "WebKit.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {
    class LegacyWebArchive;
}

class WebArchive : public IWebArchive
{
public:
    static WebArchive* createInstance();
    static WebArchive* createInstance(PassRefPtr<WebCore::LegacyWebArchive>);
protected:
    WebArchive(PassRefPtr<WebCore::LegacyWebArchive>);
    ~WebArchive();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef();
    virtual ULONG STDMETHODCALLTYPE Release();

    // IWebArchive
    virtual HRESULT STDMETHODCALLTYPE initWithMainResource(
        /* [in] */ IWebResource* mainResource, 
        /* [in, size_is(cSubResources)] */ IWebResource** subResources, 
        /* [in] */ int cSubResources, 
        /* in, size_is(cSubFrameArchives)] */ IWebArchive** subFrameArchives, 
        /* [in] */ int cSubFrameArchives);

    virtual HRESULT STDMETHODCALLTYPE  initWithData(
        /* [in] */ IStream*);

    virtual HRESULT STDMETHODCALLTYPE  initWithNode(
        /* [in] */ IDOMNode*);

    virtual HRESULT STDMETHODCALLTYPE  mainResource(
        /* [out, retval] */ IWebResource**);

    virtual HRESULT STDMETHODCALLTYPE  subResources(
        /* [out, retval] */ IEnumVARIANT**);

    virtual HRESULT STDMETHODCALLTYPE  subframeArchives(
        /* [out, retval] */ IEnumVARIANT**);

    virtual HRESULT STDMETHODCALLTYPE  data(
        /* [out, retval] */ IStream**);

protected:
    ULONG m_refCount;
    RefPtr<WebCore::LegacyWebArchive> m_archive;
};

#endif // WebArchive_h
