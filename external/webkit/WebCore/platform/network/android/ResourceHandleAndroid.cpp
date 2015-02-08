

#include "config.h"

#include "ResourceHandle.h"

#include "CString.h"
#include "DocLoader.h"
#include "DocumentLoader.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "MainResourceLoader.h"
#include "NotImplemented.h"
#include "ResourceHandleClient.h"
#include "ResourceHandleInternal.h"
#include "ResourceLoaderAndroid.h"

extern void webkit_logcat(const char *fmt, ...);

namespace WebCore {

ResourceHandleInternal::~ResourceHandleInternal()
{
}

ResourceHandle::~ResourceHandle()
{
}

bool ResourceHandle::start(Frame* frame)
{
    DocumentLoader* docLoader = frame->loader()->activeDocumentLoader();

    if (!docLoader) {
        webkit_logcat("ResourceHandle::start() failed due to docLoader is 0");
        return false;
    }

    MainResourceLoader* mainLoader = docLoader->mainResourceLoader();
    bool isMainResource =
            static_cast<void*>(mainLoader) == static_cast<void*>(client());

    PassRefPtr<ResourceLoaderAndroid> loader = ResourceLoaderAndroid::start(this, d->m_request, frame->loader()->client(), isMainResource, false);

    if (loader) {
        d->m_loader = loader;
        return true;
    }

    return false;
}

void ResourceHandle::cancel()
{
    if (d->m_loader)
        d->m_loader->cancel();
}

PassRefPtr<SharedBuffer> ResourceHandle::bufferedData()
{
    notImplemented();
    return 0;
}

bool ResourceHandle::supportsBufferedData()
{
    // We don't support buffering data on the native side.
    notImplemented();
    return false;
}

#if PLATFORM(ANDROID)
// TODO: this needs upstreaming.
void ResourceHandle::pauseLoad(bool pause)
{
    if (d->m_loader)
        d->m_loader->pauseLoad(pause);
}
#endif

void ResourceHandle::setDefersLoading(bool defers)
{
    notImplemented();
}

// This static method is called to check to see if a POST response is in
// the cache. The JNI call through to the HTTP cache stored on the Java
// side may be slow, but is only used during a navigation to
// a POST response.
bool ResourceHandle::willLoadFromCache(ResourceRequest& request, Frame*)
{
    // set the cache policy correctly, copied from
    // network/mac/ResourceHandleMac.mm
    request.setCachePolicy(ReturnCacheDataDontLoad);
    FormData* formData = request.httpBody();
    return ResourceLoaderAndroid::willLoadFromCache(request.url(), formData ? formData->identifier() : 0);
}

bool ResourceHandle::loadsBlocked() 
{
    // FIXME, need to check whether connection pipe is blocked.
    // return false for now
    return false; 
}

// Class to handle synchronized loading of resources.
class SyncLoader : public ResourceHandleClient {
public:
    SyncLoader(ResourceError& error, ResourceResponse& response, WTF::Vector<char>& data)
    {
        m_error = &error;
        m_response = &response;
        m_data = &data;
    }
    ~SyncLoader() {}

    virtual void didReceiveResponse(ResourceHandle*, const ResourceResponse& response)
    {
        *m_response = response;
    }

    virtual void didReceiveData(ResourceHandle*, const char* data, int len, int lengthReceived)
    {
        m_data->append(data, len);
    }

    virtual void didFail(ResourceHandle*, const ResourceError& error)
    {
        *m_error = error;
    }

private:
    ResourceError* m_error;
    ResourceResponse* m_response;
    WTF::Vector<char>* m_data;
};

void ResourceHandle::loadResourceSynchronously(const ResourceRequest& request,
        StoredCredentials /*storedCredentials*/,
        ResourceError& error, ResourceResponse& response, WTF::Vector<char>& data,
        Frame* frame) 
{
    SyncLoader s(error, response, data);
    ResourceHandle h(request, &s, false, false, false);
    // This blocks until the load is finished.
    // Use the request owned by the ResourceHandle. This has had the username
    // and password (if present) stripped from the URL in
    // ResourceHandleInternal::ResourceHandleInternal(). This matches the
    // behaviour in the asynchronous case.
    ResourceLoaderAndroid::start(&h, h.getInternal()->m_request, frame->loader()->client(), false, true);
}

} // namespace WebCore
