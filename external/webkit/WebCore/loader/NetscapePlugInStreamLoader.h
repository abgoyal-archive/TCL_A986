

#include "ResourceLoader.h"
#include <wtf/Forward.h>

namespace WebCore {

    class NetscapePlugInStreamLoader;

    class NetscapePlugInStreamLoaderClient {
    public:
        virtual void didReceiveResponse(NetscapePlugInStreamLoader*, const ResourceResponse&) = 0;
        virtual void didReceiveData(NetscapePlugInStreamLoader*, const char*, int) = 0;
        virtual void didFail(NetscapePlugInStreamLoader*, const ResourceError&) = 0;
        virtual void didFinishLoading(NetscapePlugInStreamLoader*) { }
        virtual bool wantsAllStreams() const { return false; }

    protected:
        virtual ~NetscapePlugInStreamLoaderClient() { }
    };

    class NetscapePlugInStreamLoader : public ResourceLoader {
    public:
        static PassRefPtr<NetscapePlugInStreamLoader> create(Frame*, NetscapePlugInStreamLoaderClient*);
        virtual ~NetscapePlugInStreamLoader();

        bool isDone() const;

    private:
        virtual void didReceiveResponse(const ResourceResponse&);
        virtual void didReceiveData(const char*, int, long long lengthReceived, bool allAtOnce);
        virtual void didFinishLoading();
        virtual void didFail(const ResourceError&);

        virtual void releaseResources();

        NetscapePlugInStreamLoader(Frame*, NetscapePlugInStreamLoaderClient*);

        virtual void didCancel(const ResourceError& error);

        NetscapePlugInStreamLoaderClient* m_client;
    };

}
