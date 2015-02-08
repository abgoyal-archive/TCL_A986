

#include "config.h"
#include "ResourceHandle.h"

#include "ResourceHandleClient.h"
#include "ResourceRequest.h"

#include "WebKit.h"
#include "WebKitClient.h"
#include "WebURLError.h"
#include "WebURLLoader.h"
#include "WebURLLoaderClient.h"
#include "WebURLRequest.h"
#include "WebURLResponse.h"
#include "WrappedResourceRequest.h"
#include "WrappedResourceResponse.h"

using namespace WebKit;

namespace WebCore {

// ResourceHandleInternal -----------------------------------------------------

class ResourceHandleInternal : public WebURLLoaderClient {
public:
    ResourceHandleInternal(const ResourceRequest& request, ResourceHandleClient* client)
        : m_request(request)
        , m_owner(0)
        , m_client(client)
    {
    }

    void start();
    void cancel();
    void setDefersLoading(bool);
    bool allowStoredCredentials() const;

    // WebURLLoaderClient methods:
    virtual void willSendRequest(WebURLLoader*, WebURLRequest&, const WebURLResponse&);
    virtual void didSendData(
        WebURLLoader*, unsigned long long bytesSent, unsigned long long totalBytesToBeSent);
    virtual void didReceiveResponse(WebURLLoader*, const WebURLResponse&);
    virtual void didReceiveData(WebURLLoader*, const char* data, int dataLength);
    virtual void didFinishLoading(WebURLLoader*);
    virtual void didFail(WebURLLoader*, const WebURLError&);

    ResourceRequest m_request;
    ResourceHandle* m_owner;
    ResourceHandleClient* m_client;
    OwnPtr<WebURLLoader> m_loader;
};

void ResourceHandleInternal::start()
{
    m_loader.set(webKitClient()->createURLLoader());
    ASSERT(m_loader.get());

    WrappedResourceRequest wrappedRequest(m_request);
    wrappedRequest.setAllowStoredCredentials(allowStoredCredentials());
    m_loader->loadAsynchronously(wrappedRequest, this);
}

void ResourceHandleInternal::cancel()
{
    m_loader->cancel();

    // Do not make any further calls to the client.
    m_client = 0;
}

void ResourceHandleInternal::setDefersLoading(bool value)
{
    m_loader->setDefersLoading(value);
}

bool ResourceHandleInternal::allowStoredCredentials() const
{
    return m_client && m_client->shouldUseCredentialStorage(m_owner);
}

void ResourceHandleInternal::willSendRequest(
    WebURLLoader*, WebURLRequest& request, const WebURLResponse& response)
{
    ASSERT(m_client);
    ASSERT(!request.isNull());
    ASSERT(!response.isNull());
    m_client->willSendRequest(m_owner, request.toMutableResourceRequest(), response.toResourceResponse());
}

void ResourceHandleInternal::didSendData(
    WebURLLoader*, unsigned long long bytesSent, unsigned long long totalBytesToBeSent)
{
    ASSERT(m_client);
    m_client->didSendData(m_owner, bytesSent, totalBytesToBeSent);
}

void ResourceHandleInternal::didReceiveResponse(WebURLLoader*, const WebURLResponse& response)
{
    ASSERT(m_client);
    ASSERT(!response.isNull());
    m_client->didReceiveResponse(m_owner, response.toResourceResponse());
}

void ResourceHandleInternal::didReceiveData(
    WebURLLoader*, const char* data, int dataLength)
{
    ASSERT(m_client);

    // FIXME(yurys): it looks like lengthReceived is always the same as
    // dataLength and that the latter parameter can be eliminated.
    // See WebKit bug: https://bugs.webkit.org/show_bug.cgi?id=31019
    m_client->didReceiveData(m_owner, data, dataLength, dataLength);
}

void ResourceHandleInternal::didFinishLoading(WebURLLoader*)
{
    ASSERT(m_client);
    m_client->didFinishLoading(m_owner);
}

void ResourceHandleInternal::didFail(WebURLLoader*, const WebURLError& error)
{
    ASSERT(m_client);
    m_client->didFail(m_owner, error);
}

// ResourceHandle -------------------------------------------------------------

ResourceHandle::ResourceHandle(const ResourceRequest& request,
                               ResourceHandleClient* client,
                               bool defersLoading,
                               bool shouldContentSniff,
                               bool mightDownloadFromHandle)
    : d(new ResourceHandleInternal(request, client))
{
    d->m_owner = this;

    // FIXME: Figure out what to do with the bool params.
}

PassRefPtr<ResourceHandle> ResourceHandle::create(const ResourceRequest& request,
                                                  ResourceHandleClient* client,
                                                  Frame* deprecated,
                                                  bool defersLoading,
                                                  bool shouldContentSniff,
                                                  bool mightDownloadFromHandle)
{
    RefPtr<ResourceHandle> newHandle = adoptRef(new ResourceHandle(
        request, client, defersLoading, shouldContentSniff, mightDownloadFromHandle));

    if (newHandle->start(deprecated))
        return newHandle.release();

    return 0;
}

const ResourceRequest& ResourceHandle::request() const
{
    return d->m_request;
}

ResourceHandleClient* ResourceHandle::client() const
{
    return d->m_client;
}

void ResourceHandle::setClient(ResourceHandleClient* client)
{
    d->m_client = client;
}

void ResourceHandle::setDefersLoading(bool value)
{
    d->setDefersLoading(value);
}

bool ResourceHandle::start(Frame* deprecated)
{
    d->start();
    return true;
}

void ResourceHandle::clearAuthentication()
{
}

void ResourceHandle::cancel()
{
    d->cancel();
}

ResourceHandle::~ResourceHandle()
{
    d->m_owner = 0;
}

PassRefPtr<SharedBuffer> ResourceHandle::bufferedData()
{
    return 0;
}

bool ResourceHandle::loadsBlocked()
{
    return false;  // This seems to be related to sync XMLHttpRequest...
}

// static
bool ResourceHandle::supportsBufferedData()
{
    return false;  // The loader will buffer manually if it needs to.
}

// static
void ResourceHandle::loadResourceSynchronously(const ResourceRequest& request,
                                               StoredCredentials storedCredentials,
                                               ResourceError& error,
                                               ResourceResponse& response,
                                               Vector<char>& data,
                                               Frame* deprecated)
{
    OwnPtr<WebURLLoader> loader(webKitClient()->createURLLoader());
    ASSERT(loader.get());

    WrappedResourceRequest requestIn(request);
    requestIn.setAllowStoredCredentials(storedCredentials == AllowStoredCredentials);
    WrappedResourceResponse responseOut(response);
    WebURLError errorOut;
    WebData dataOut;

    loader->loadSynchronously(requestIn, responseOut, errorOut, dataOut);

    error = errorOut;
    data.clear();
    data.append(dataOut.data(), dataOut.size());
}

// static
bool ResourceHandle::willLoadFromCache(ResourceRequest& request, Frame*)
{
    // This method is used to determine if a POST request can be repeated from
    // cache, but you cannot really know until you actually try to read from the
    // cache.  Even if we checked now, something else could come along and wipe
    // out the cache entry by the time we fetch it.
    //
    // So, we always say yes here, to prevent the FrameLoader from initiating a
    // reload.  Then in FrameLoaderClientImpl::dispatchWillSendRequest, we
    // fix-up the cache policy of the request to force a load from the cache.
    //
    ASSERT(request.httpMethod() == "POST");
    return true;
}

} // namespace WebCore
