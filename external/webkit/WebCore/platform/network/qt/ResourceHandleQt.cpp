

#include "config.h"

#include "Frame.h"
#include "DocLoader.h"
#include "ResourceHandle.h"
#include "ResourceHandleClient.h"
#include "ResourceHandleInternal.h"
#include "qwebpage_p.h"
#include "qwebframe_p.h"
#include "ChromeClientQt.h"
#include "FrameLoaderClientQt.h"
#include "Page.h"
#include "QNetworkReplyHandler.h"

#include "NotImplemented.h"

#if QT_VERSION >= 0x040500
#include <QAbstractNetworkCache>
#endif
#include <QCoreApplication>
#include <QUrl>
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>

namespace WebCore {

class WebCoreSynchronousLoader : public ResourceHandleClient {
public:
    WebCoreSynchronousLoader();

    void waitForCompletion();

    virtual void didReceiveResponse(ResourceHandle*, const ResourceResponse&);
    virtual void didReceiveData(ResourceHandle*, const char*, int, int lengthReceived);
    virtual void didFinishLoading(ResourceHandle*);
    virtual void didFail(ResourceHandle*, const ResourceError&);

    ResourceResponse resourceResponse() const { return m_response; }
    ResourceError resourceError() const { return m_error; }
    Vector<char> data() const { return m_data; }

private:
    ResourceResponse m_response;
    ResourceError m_error;
    Vector<char> m_data;
    QEventLoop m_eventLoop;
};

WebCoreSynchronousLoader::WebCoreSynchronousLoader()
{
}

void WebCoreSynchronousLoader::didReceiveResponse(ResourceHandle*, const ResourceResponse& response)
{
    m_response = response;
}

void WebCoreSynchronousLoader::didReceiveData(ResourceHandle*, const char* data, int length, int)
{
    m_data.append(data, length);
}

void WebCoreSynchronousLoader::didFinishLoading(ResourceHandle*)
{
    m_eventLoop.exit();
}

void WebCoreSynchronousLoader::didFail(ResourceHandle*, const ResourceError& error)
{
    m_error = error;
    m_eventLoop.exit();
}

void WebCoreSynchronousLoader::waitForCompletion()
{
    m_eventLoop.exec(QEventLoop::ExcludeUserInputEvents);
}

ResourceHandleInternal::~ResourceHandleInternal()
{
}

ResourceHandle::~ResourceHandle()
{
    if (d->m_job)
        cancel();
}

bool ResourceHandle::start(Frame* frame)
{
    if (!frame)
        return false;

    Page *page = frame->page();
    // If we are no longer attached to a Page, this must be an attempted load from an
    // onUnload handler, so let's just block it.
    if (!page)
        return false;

    if (!(d->m_user.isEmpty() || d->m_pass.isEmpty())) {
        // If credentials were specified for this request, add them to the url,
        // so that they will be passed to QNetworkRequest.
        KURL urlWithCredentials(d->m_request.url());
        urlWithCredentials.setUser(d->m_user);
        urlWithCredentials.setPass(d->m_pass);
        d->m_request.setURL(urlWithCredentials);
    }

    getInternal()->m_frame = static_cast<FrameLoaderClientQt*>(frame->loader()->client())->webFrame();
    ResourceHandleInternal *d = getInternal();
    d->m_job = new QNetworkReplyHandler(this, QNetworkReplyHandler::LoadMode(d->m_defersLoading));
    return true;
}

void ResourceHandle::cancel()
{
    if (d->m_job)
        d->m_job->abort();
}

bool ResourceHandle::loadsBlocked()
{
    return false;
}

bool ResourceHandle::willLoadFromCache(ResourceRequest& request, Frame* frame)
{
    if (!frame)
        return false;

#if QT_VERSION >= 0x040500
    QNetworkAccessManager* manager = QWebFramePrivate::kit(frame)->page()->networkAccessManager();
    QAbstractNetworkCache* cache = manager->cache();

    if (!cache)
        return false;

    QNetworkCacheMetaData data = cache->metaData(request.url());
    if (data.isValid()) {
        request.setCachePolicy(ReturnCacheDataDontLoad);
        return true;
    }

    return false;
#else
    return false;
#endif
}

bool ResourceHandle::supportsBufferedData()
{
    return false;
}

PassRefPtr<SharedBuffer> ResourceHandle::bufferedData()
{
    ASSERT_NOT_REACHED();
    return 0;
}

void ResourceHandle::loadResourceSynchronously(const ResourceRequest& request, StoredCredentials /*storedCredentials*/, ResourceError& error, ResourceResponse& response, Vector<char>& data, Frame* frame)
{
    WebCoreSynchronousLoader syncLoader;
    ResourceHandle handle(request, &syncLoader, true, false, true);

    ResourceHandleInternal *d = handle.getInternal();
    if (!(d->m_user.isEmpty() || d->m_pass.isEmpty())) {
        // If credentials were specified for this request, add them to the url,
        // so that they will be passed to QNetworkRequest.
        KURL urlWithCredentials(d->m_request.url());
        urlWithCredentials.setUser(d->m_user);
        urlWithCredentials.setPass(d->m_pass);
        d->m_request.setURL(urlWithCredentials);
    }
    d->m_frame = static_cast<FrameLoaderClientQt*>(frame->loader()->client())->webFrame();
    d->m_job = new QNetworkReplyHandler(&handle, QNetworkReplyHandler::LoadNormal);

    syncLoader.waitForCompletion();
    error = syncLoader.resourceError();
    data = syncLoader.data();
    response = syncLoader.resourceResponse();
}

 
void ResourceHandle::setDefersLoading(bool defers)
{
    d->m_defersLoading = defers;

    if (d->m_job)
        d->m_job->setLoadMode(QNetworkReplyHandler::LoadMode(defers));
}

} // namespace WebCore
