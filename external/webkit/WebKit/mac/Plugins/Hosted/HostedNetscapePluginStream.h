

#if USE(PLUGIN_HOST_PROCESS)

#ifndef HostedNetscapePluginStream_h
#define HostedNetscapePluginStream_h

#include <WebCore/NetscapePlugInStreamLoader.h>
#include <WebKit/npapi.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>
#include <wtf/RetainPtr.h>

namespace WebCore {
    class FrameLoader;
    class NetscapePlugInStreamLoader;
}

namespace WebKit {

class NetscapePluginInstanceProxy;

class HostedNetscapePluginStream : public RefCounted<HostedNetscapePluginStream>
                                 , private WebCore::NetscapePlugInStreamLoaderClient {
public:
    static PassRefPtr<HostedNetscapePluginStream> create(NetscapePluginInstanceProxy* instance, uint32_t streamID, NSURLRequest *request)
    {
        return adoptRef(new HostedNetscapePluginStream(instance, streamID, request));
    }
    static PassRefPtr<HostedNetscapePluginStream> create(NetscapePluginInstanceProxy* instance, WebCore::FrameLoader* frameLoader)
    {
        return adoptRef(new HostedNetscapePluginStream(instance, frameLoader));
    }

    ~HostedNetscapePluginStream();

    uint32_t streamID() const { return m_streamID; }

    void startStreamWithResponse(NSURLResponse *response);
    void didReceiveData(WebCore::NetscapePlugInStreamLoader*, const char* bytes, int length);
    void didFinishLoading(WebCore::NetscapePlugInStreamLoader*);
    void didFail(WebCore::NetscapePlugInStreamLoader*, const WebCore::ResourceError&);

    void start();
    void stop();

    void cancelLoad(NPReason reason);

    static NPReason reasonForError(NSError* error);

private:
    NSError *errorForReason(NPReason) const;
    void cancelLoad(NSError *);

    HostedNetscapePluginStream(NetscapePluginInstanceProxy*, uint32_t streamID, NSURLRequest *);
    HostedNetscapePluginStream(NetscapePluginInstanceProxy*, WebCore::FrameLoader*);
    
    void startStream(NSURL *, long long expectedContentLength, NSDate *lastModifiedDate, NSString *mimeType, NSData *headers);

    NSError *pluginCancelledConnectionError() const;

    // NetscapePlugInStreamLoaderClient methods.
    void didReceiveResponse(WebCore::NetscapePlugInStreamLoader*, const WebCore::ResourceResponse&);
    bool wantsAllStreams() const;
    
    RefPtr<NetscapePluginInstanceProxy> m_instance;
    uint32_t m_streamID;
    bool m_isTerminated;
    RetainPtr<NSMutableURLRequest> m_request;

    RetainPtr<NSURL> m_requestURL;
    RetainPtr<NSURL> m_responseURL;
    RetainPtr<NSString> m_mimeType;

    WebCore::FrameLoader* m_frameLoader;
    RefPtr<WebCore::NetscapePlugInStreamLoader> m_loader;
};

}

#endif // HostedNetscapePluginStream_h
#endif // USE(PLUGIN_HOST_PROCESS)
