

#include "config.h"

#if ENABLE(WORKERS)

#include "WorkerScriptLoader.h"

#include "GenericWorkerTask.h"
#include "ResourceRequest.h"
#include "ScriptExecutionContext.h"
#include "SecurityOrigin.h"
#include "WorkerContext.h"
#include "WorkerScriptLoaderClient.h"
#include "WorkerThreadableLoader.h"
#include <wtf/OwnPtr.h>
#include <wtf/UnusedParam.h>

namespace WebCore {

WorkerScriptLoader::WorkerScriptLoader()
    : m_client(0)
    , m_failed(false)
    , m_identifier(0)
{
}

void WorkerScriptLoader::loadSynchronously(ScriptExecutionContext* scriptExecutionContext, const KURL& url, CrossOriginRequestPolicy crossOriginRequestPolicy)
{
    m_url = url;

    OwnPtr<ResourceRequest> request(createResourceRequest());
    if (!request)
        return;

    ASSERT(scriptExecutionContext->isWorkerContext());

    ThreadableLoaderOptions options;
    options.allowCredentials = true;
    options.crossOriginRequestPolicy = crossOriginRequestPolicy;

    WorkerThreadableLoader::loadResourceSynchronously(static_cast<WorkerContext*>(scriptExecutionContext), *request, *this, options);
}
    
void WorkerScriptLoader::loadAsynchronously(ScriptExecutionContext* scriptExecutionContext, const KURL& url, CrossOriginRequestPolicy crossOriginRequestPolicy, WorkerScriptLoaderClient* client)
{
    ASSERT(client);
    m_client = client;
    m_url = url;

    OwnPtr<ResourceRequest> request(createResourceRequest());
    if (!request)
        return;

    ThreadableLoaderOptions options;
    options.allowCredentials = true;
    options.crossOriginRequestPolicy = crossOriginRequestPolicy;

    m_threadableLoader = ThreadableLoader::create(scriptExecutionContext, this, *request, options);
}

PassOwnPtr<ResourceRequest> WorkerScriptLoader::createResourceRequest()
{
    OwnPtr<ResourceRequest> request(new ResourceRequest(m_url));
    request->setHTTPMethod("GET");

    return request.release();
}
    
void WorkerScriptLoader::didReceiveResponse(const ResourceResponse& response)
{
    if (response.httpStatusCode() / 100 != 2 && response.httpStatusCode()) {
        m_failed = true;
        return;
    }
    m_responseEncoding = response.textEncodingName();
}

void WorkerScriptLoader::didReceiveData(const char* data, int len)
{
    if (m_failed)
        return;

    if (!m_decoder) {
        if (!m_responseEncoding.isEmpty())
            m_decoder = TextResourceDecoder::create("text/javascript", m_responseEncoding);
        else
            m_decoder = TextResourceDecoder::create("text/javascript", "UTF-8");
    }

    if (!len)
        return;
    
    if (len == -1)
        len = strlen(data);
    
    m_script += m_decoder->decode(data, len);
}

void WorkerScriptLoader::didFinishLoading(unsigned long identifier)
{
    if (m_failed)
        return;

    if (m_decoder)
        m_script += m_decoder->flush();

    m_identifier = identifier;
    notifyFinished();
}

void WorkerScriptLoader::didFail(const ResourceError&)
{
    notifyError();
}

void WorkerScriptLoader::didFailRedirectCheck()
{
    notifyError();
}

void WorkerScriptLoader::didReceiveAuthenticationCancellation(const ResourceResponse&)
{
    notifyError();
}

void WorkerScriptLoader::notifyError()
{
    m_failed = true;
    notifyFinished();
}
    
void WorkerScriptLoader::notifyFinished()
{
    if (m_client)
        m_client->notifyFinished();
}

} // namespace WebCore

#endif // ENABLE(WORKERS)
