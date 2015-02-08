

#include "config.h"
#include "WebKitDLL.h"
#include "WebDownload.h"

#include "CString.h"
#include "DefaultDownloadDelegate.h"
#include "MarshallingHelpers.h"
#include "WebError.h"
#include "WebKit.h"
#include "WebKitLogging.h"
#include "WebMutableURLRequest.h"
#include "WebURLAuthenticationChallenge.h"
#include "WebURLCredential.h"
#include "WebURLResponse.h"

#include <wtf/platform.h>

#include <io.h>
#include <sys/stat.h>
#include <sys/types.h>

#pragma warning(push, 0)
#include <WebCore/BString.h>
#include <WebCore/NotImplemented.h>
#include <WebCore/ResourceError.h>
#include <WebCore/ResourceHandle.h>
#include <WebCore/ResourceRequest.h>
#include <WebCore/ResourceResponse.h>
#include <WebCore/SystemTime.h>
#pragma warning(pop)

using namespace WebCore;

// WebDownload ----------------------------------------------------------------

void WebDownload::init(ResourceHandle* handle, const ResourceRequest& request, const ResourceResponse& response, IWebDownloadDelegate* delegate)
{
   notImplemented();
}

void WebDownload::init(const KURL& url, IWebDownloadDelegate* delegate)
{
   notImplemented();
}

// IWebDownload -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebDownload::initWithRequest(
        /* [in] */ IWebURLRequest* request, 
        /* [in] */ IWebDownloadDelegate* delegate)
{
   notImplemented();
   return E_FAIL;
}

HRESULT STDMETHODCALLTYPE WebDownload::initToResumeWithBundle(
        /* [in] */ BSTR bundlePath, 
        /* [in] */ IWebDownloadDelegate* delegate)
{
   notImplemented();
   return E_FAIL;
}

HRESULT STDMETHODCALLTYPE WebDownload::start()
{
   notImplemented();
   return E_FAIL;
}

HRESULT STDMETHODCALLTYPE WebDownload::cancel()
{
   notImplemented();
   return E_FAIL;
}

HRESULT STDMETHODCALLTYPE WebDownload::cancelForResume()
{
   notImplemented();
   return E_FAIL;
}

HRESULT STDMETHODCALLTYPE WebDownload::deletesFileUponFailure(
        /* [out, retval] */ BOOL* result)
{
   notImplemented();
   return E_FAIL;
}

HRESULT STDMETHODCALLTYPE WebDownload::setDeletesFileUponFailure(
        /* [in] */ BOOL deletesFileUponFailure)
{
   notImplemented();
   return E_FAIL;
}

HRESULT STDMETHODCALLTYPE WebDownload::setDestination(
        /* [in] */ BSTR path, 
        /* [in] */ BOOL allowOverwrite)
{
   notImplemented();
   return E_FAIL;
}

// IWebURLAuthenticationChallengeSender -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE WebDownload::cancelAuthenticationChallenge(
        /* [in] */ IWebURLAuthenticationChallenge*)
{
   notImplemented();
   return E_FAIL;
}

HRESULT STDMETHODCALLTYPE WebDownload::continueWithoutCredentialForAuthenticationChallenge(
        /* [in] */ IWebURLAuthenticationChallenge* challenge)
{
   notImplemented();
   return E_FAIL;
}

HRESULT STDMETHODCALLTYPE WebDownload::useCredential(
        /* [in] */ IWebURLCredential* credential, 
        /* [in] */ IWebURLAuthenticationChallenge* challenge)
{
   notImplemented();
   return E_FAIL;
}
