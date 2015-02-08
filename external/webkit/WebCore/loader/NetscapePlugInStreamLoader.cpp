

#include "config.h"
#include "NetscapePlugInStreamLoader.h"

#include "FrameLoader.h"
#include "DocumentLoader.h"

namespace WebCore {

NetscapePlugInStreamLoader::NetscapePlugInStreamLoader(Frame* frame, NetscapePlugInStreamLoaderClient* client)
    : ResourceLoader(frame, true, true)
    , m_client(client)
{
}

NetscapePlugInStreamLoader::~NetscapePlugInStreamLoader()
{
}

PassRefPtr<NetscapePlugInStreamLoader> NetscapePlugInStreamLoader::create(Frame* frame, NetscapePlugInStreamLoaderClient* client)
{
    return adoptRef(new NetscapePlugInStreamLoader(frame, client));
}

bool NetscapePlugInStreamLoader::isDone() const
{
    return !m_client;
}

void NetscapePlugInStreamLoader::releaseResources()
{
    m_client = 0;
    ResourceLoader::releaseResources();
}

void NetscapePlugInStreamLoader::didReceiveResponse(const ResourceResponse& response)
{
    RefPtr<NetscapePlugInStreamLoader> protect(this);

    m_client->didReceiveResponse(this, response);

    // Don't continue if the stream is cancelled
    if (!m_client)
        return;

    ResourceLoader::didReceiveResponse(response);

    // Don't continue if the stream is cancelled
    if (!m_client)
        return;

    if (!response.isHTTP())
        return;
    
    if (m_client->wantsAllStreams())
        return;
    
    if (response.httpStatusCode() < 100 || response.httpStatusCode() >= 400)
        didCancel(frameLoader()->fileDoesNotExistError(response));
}

void NetscapePlugInStreamLoader::didReceiveData(const char* data, int length, long long lengthReceived, bool allAtOnce)
{
    RefPtr<NetscapePlugInStreamLoader> protect(this);

    m_client->didReceiveData(this, data, length);
    
    ResourceLoader::didReceiveData(data, length, lengthReceived, allAtOnce);
}

void NetscapePlugInStreamLoader::didFinishLoading()
{
    RefPtr<NetscapePlugInStreamLoader> protect(this);

    m_documentLoader->removePlugInStreamLoader(this);
    m_client->didFinishLoading(this);
    ResourceLoader::didFinishLoading();
}

void NetscapePlugInStreamLoader::didFail(const ResourceError& error)
{
    RefPtr<NetscapePlugInStreamLoader> protect(this);

    m_documentLoader->removePlugInStreamLoader(this);
    m_client->didFail(this, error);
    ResourceLoader::didFail(error);
}

void NetscapePlugInStreamLoader::didCancel(const ResourceError& error)
{
    RefPtr<NetscapePlugInStreamLoader> protect(this);

    m_client->didFail(this, error);

    // If calling didFail spins the run loop the stream loader can reach the terminal state.
    // If that's the case we just return early.
    if (reachedTerminalState())
        return;
    
    // We need to remove the stream loader after the call to didFail, since didFail can 
    // spawn a new run loop and if the loader has been removed it won't be deferred when
    // the document loader is asked to defer loading.
    m_documentLoader->removePlugInStreamLoader(this);

    ResourceLoader::didCancel(error);
}

}
