

#ifndef WorkerScriptLoader_h
#define WorkerScriptLoader_h

#if ENABLE(WORKERS)

#include "KURL.h"
#include "ResourceResponse.h"
#include "ScriptString.h"
#include "TextResourceDecoder.h"
#include "ThreadableLoader.h"
#include "ThreadableLoaderClient.h"

namespace WebCore {

    class ScriptExecutionContext;
    class WorkerScriptLoaderClient;

    class WorkerScriptLoader : public ThreadableLoaderClient {
    public:
        WorkerScriptLoader();

        void loadSynchronously(ScriptExecutionContext*, const KURL&, CrossOriginRequestPolicy);
        void loadAsynchronously(ScriptExecutionContext*, const KURL&, CrossOriginRequestPolicy, WorkerScriptLoaderClient*);

        void notifyError();

        const String& script() const { return m_script; }
        const KURL& url() const { return m_url; }
        bool failed() const { return m_failed; }
        unsigned long identifier() const { return m_identifier; }

        virtual void didReceiveResponse(const ResourceResponse&);
        virtual void didReceiveData(const char* data, int lengthReceived);
        virtual void didFinishLoading(unsigned long identifier);
        virtual void didFail(const ResourceError&);
        virtual void didFailRedirectCheck();
        virtual void didReceiveAuthenticationCancellation(const ResourceResponse&);

    private:
        PassOwnPtr<ResourceRequest> createResourceRequest();
        void notifyFinished();

        WorkerScriptLoaderClient* m_client;
        RefPtr<ThreadableLoader> m_threadableLoader;
        String m_responseEncoding;        
        RefPtr<TextResourceDecoder> m_decoder;
        String m_script;
        KURL m_url;
        bool m_failed;
        unsigned long m_identifier;
    };

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // WorkerScriptLoader_h
