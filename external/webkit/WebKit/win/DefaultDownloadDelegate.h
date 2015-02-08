
#ifndef DefaultDownloadDelegate_h
#define DefaultDownloadDelegate_h

#include "COMPtr.h"
#include "WebKit.h"
#include <wtf/HashSet.h>

#if USE(CFNETWORK)
#include <CFNetwork/CFURLDownloadPriv.h>
#endif

class DefaultDownloadDelegate : public IWebDownloadDelegate
{
public:
    static DefaultDownloadDelegate* sharedInstance();
    static DefaultDownloadDelegate* createInstance();
private:
    DefaultDownloadDelegate();
    ~DefaultDownloadDelegate();
public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebDownloadDelegate
    virtual HRESULT STDMETHODCALLTYPE decideDestinationWithSuggestedFilename(IWebDownload *download, BSTR filename);
    virtual HRESULT STDMETHODCALLTYPE didCancelAuthenticationChallenge(IWebDownload* download,  IWebURLAuthenticationChallenge* challenge);
    virtual HRESULT STDMETHODCALLTYPE didCreateDestination(IWebDownload* download,  BSTR destination);
    virtual HRESULT STDMETHODCALLTYPE didFailWithError(IWebDownload* download,  IWebError* error);
    virtual HRESULT STDMETHODCALLTYPE didReceiveAuthenticationChallenge(IWebDownload* download,  IWebURLAuthenticationChallenge* challenge);
    virtual HRESULT STDMETHODCALLTYPE didReceiveDataOfLength(IWebDownload* download,  unsigned length);
    virtual HRESULT STDMETHODCALLTYPE didReceiveResponse(IWebDownload* download,  IWebURLResponse* response);
    virtual HRESULT STDMETHODCALLTYPE shouldDecodeSourceDataOfMIMEType(IWebDownload* download,  BSTR encodingType, BOOL* shouldDecode);
    virtual HRESULT STDMETHODCALLTYPE willResumeWithResponse(IWebDownload* download,  IWebURLResponse* response,  long long fromByte);
    virtual HRESULT STDMETHODCALLTYPE willSendRequest(IWebDownload* download, IWebMutableURLRequest* request,  IWebURLResponse* redirectResponse, IWebMutableURLRequest** finalRequest);
    virtual HRESULT STDMETHODCALLTYPE didBegin(IWebDownload* download);
    virtual HRESULT STDMETHODCALLTYPE didFinish(IWebDownload* download);

    // DefaultDownloadDelegate
    void registerDownload(IWebDownload*);
    void unregisterDownload(IWebDownload*);
protected:
    ULONG m_refCount;

    HashSet<IWebDownload*> m_downloads;
};

#endif
