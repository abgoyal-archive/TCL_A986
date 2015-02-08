
 
#include "config.h"
#include "IconLoader.h"

#include "Document.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "FrameLoaderClient.h"
#include "IconDatabase.h"
#include "Logging.h"
#include "ResourceHandle.h"
#include "ResourceResponse.h"
#include "ResourceRequest.h"
#include "SharedBuffer.h"
#include "SubresourceLoader.h"
#include <wtf/UnusedParam.h>

using namespace std;

namespace WebCore {

IconLoader::IconLoader(Frame* frame)
    : m_frame(frame)
    , m_loadIsInProgress(false)
{
}

auto_ptr<IconLoader> IconLoader::create(Frame* frame)
{
    return auto_ptr<IconLoader>(new IconLoader(frame));
}

IconLoader::~IconLoader()
{
}

void IconLoader::startLoading()
{    
    if (m_resourceLoader)
        return;

    // Set flag so we can detect the case where the load completes before
    // SubresourceLoader::create returns.
    m_loadIsInProgress = true;

    RefPtr<SubresourceLoader> loader = SubresourceLoader::create(m_frame, this, m_frame->loader()->iconURL());
    if (!loader)
        LOG_ERROR("Failed to start load for icon at url %s", m_frame->loader()->iconURL().string().ascii().data());

    // Store the handle so we can cancel the load if stopLoading is called later.
    // But only do it if the load hasn't already completed.
    if (m_loadIsInProgress)
        m_resourceLoader = loader.release();
}

void IconLoader::stopLoading()
{
    clearLoadingState();
}

void IconLoader::didReceiveResponse(SubresourceLoader* resourceLoader, const ResourceResponse& response)
{
    // If we got a status code indicating an invalid response, then lets
    // ignore the data and not try to decode the error page as an icon.
    int status = response.httpStatusCode();
    LOG(IconDatabase, "IconLoader::didReceiveResponse() - Loader %p, response %i", resourceLoader, status);

    if (status && (status < 200 || status > 299)) {
        ResourceHandle* handle = resourceLoader->handle();
        finishLoading(handle ? handle->request().url() : KURL(), 0);
    }
}

void IconLoader::didReceiveData(SubresourceLoader* unusedLoader, const char*, int unusedSize)
{
#if LOG_DISABLED
    UNUSED_PARAM(unusedLoader);
    UNUSED_PARAM(unusedSize);
#endif
    LOG(IconDatabase, "IconLoader::didReceiveData() - Loader %p, number of bytes %i", unusedLoader, unusedSize);
}

void IconLoader::didFail(SubresourceLoader* resourceLoader, const ResourceError&)
{
    LOG(IconDatabase, "IconLoader::didFail() - Loader %p", resourceLoader);
    
    // Until <rdar://problem/5463392> is resolved and we can properly cancel SubresourceLoaders when they get an error response,
    // we need to be prepared to receive this call even after we've "finished loading" once.
    // After it is resolved, we can restore an assertion that the load is in progress if ::didFail() is called
    
    if (m_loadIsInProgress) {
        ASSERT(resourceLoader == m_resourceLoader);
        ResourceHandle* handle = resourceLoader->handle();
        finishLoading(handle ? handle->request().url() : KURL(), 0);
    }
}

void IconLoader::didReceiveAuthenticationChallenge(SubresourceLoader*, const AuthenticationChallenge&)
{
    // We don't ever want to prompt for authentication just for a site icon, so
    // implement this method to cancel the resource load
    m_resourceLoader->cancel();
}

void IconLoader::didFinishLoading(SubresourceLoader* resourceLoader)
{
    LOG(IconDatabase, "IconLoader::didFinishLoading() - Loader %p", resourceLoader);

    // Until <rdar://problem/5463392> is resolved and we can properly cancel SubresourceLoaders when they get an error response,
    // we need to be prepared to receive this call even after we've "finished loading" once.
    // After it is resolved, we can restore an assertion that the load is in progress if ::didFail() is called
    
    if (m_loadIsInProgress) {
        ASSERT(resourceLoader == m_resourceLoader);
        ResourceHandle* handle = resourceLoader->handle();
        finishLoading(handle ? handle->request().url() : KURL(), m_resourceLoader->resourceData());
    }
}

void IconLoader::finishLoading(const KURL& iconURL, PassRefPtr<SharedBuffer> data)
{
    // When an icon load results in a 404 we commit it to the database here and clear the loading state.  
    // But the SubresourceLoader continues pulling in data in the background for the 404 page if the server sends one.  
    // Once that data finishes loading or if the load is cancelled while that data is being read, finishLoading ends up being called a second time.
    // We need to change SubresourceLoader to have a mode where it will stop itself after receiving a 404 so this won't happen -
    // in the meantime, we'll only commit this data to the IconDatabase if it's the first time ::finishLoading() is called
    // <rdar://problem/5463392> tracks that enhancement
    
    if (!iconURL.isEmpty() && m_loadIsInProgress) {
        LOG(IconDatabase, "IconLoader::finishLoading() - Committing iconURL %s to database", iconURL.string().ascii().data());
        m_frame->loader()->commitIconURLToIconDatabase(iconURL);
        // Setting the icon data only after committing to the database ensures that the data is
        // kept in memory (so it does not have to be read from the database asynchronously), since
        // there is a page URL referencing it.
        iconDatabase()->setIconDataForIconURL(data, iconURL.string());
        m_frame->loader()->client()->dispatchDidReceiveIcon();
    }

    clearLoadingState();
}

void IconLoader::clearLoadingState()
{
    m_resourceLoader = 0;
    m_loadIsInProgress = false;
}

}
