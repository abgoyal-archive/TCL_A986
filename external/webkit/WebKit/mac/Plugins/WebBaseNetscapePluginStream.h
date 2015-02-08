

#if ENABLE(NETSCAPE_PLUGIN_API)
#import <Foundation/Foundation.h>

#import <WebCore/Timer.h>
#import <WebCore/NetscapePlugInStreamLoader.h>
#import <WebKit/npfunctions.h>
#import <wtf/PassRefPtr.h>
#import <wtf/RefCounted.h>
#import <wtf/RefPtr.h>
#import <wtf/RetainPtr.h>

#import "WebNetscapePluginView.h"

namespace WebCore {
    class FrameLoader;
    class NetscapePlugInStreamLoader;
}

@class WebNetscapePluginView;
@class NSURLResponse;

class WebNetscapePluginStream : public RefCounted<WebNetscapePluginStream>
                              , private WebCore::NetscapePlugInStreamLoaderClient
{
public:
    static PassRefPtr<WebNetscapePluginStream> create(NSURLRequest *request, NPP plugin, bool sendNotification, void* notifyData)
    {
        return adoptRef(new WebNetscapePluginStream(request, plugin, sendNotification, notifyData));
    }
    static PassRefPtr<WebNetscapePluginStream> create(WebCore::FrameLoader* frameLoader)
    {
        return adoptRef(new WebNetscapePluginStream(frameLoader));
    }
    virtual ~WebNetscapePluginStream();

    NPP plugin() const { return m_plugin; }
    void setPlugin(NPP);
    
    static NPP ownerForStream(NPStream *);

    static NPReason reasonForError(NSError *);
    NSError *errorForReason(NPReason) const;

    void cancelLoadAndDestroyStreamWithError(NSError *);

    void setRequestURL(NSURL *requestURL) { m_requestURL = requestURL; }

    void start();
    void stop();
    
    void startStreamWithResponse(NSURLResponse *response);
    
    void didReceiveData(WebCore::NetscapePlugInStreamLoader*, const char* bytes, int length);
    void destroyStreamWithError(NSError *);
    void didFinishLoading(WebCore::NetscapePlugInStreamLoader*);

private:
    void destroyStream();
    void cancelLoadWithError(NSError *);
    void destroyStreamWithReason(NPReason);
    void deliverDataToFile(NSData *data);
    void deliverData();

    void startStream(NSURL *, long long expectedContentLength, NSDate *lastModifiedDate, NSString *mimeType, NSData *headers);
    
    NSError *pluginCancelledConnectionError() const;

    // NetscapePlugInStreamLoaderClient methods.
    void didReceiveResponse(WebCore::NetscapePlugInStreamLoader*, const WebCore::ResourceResponse&);
    void didFail(WebCore::NetscapePlugInStreamLoader*, const WebCore::ResourceError&);
    bool wantsAllStreams() const;

    RetainPtr<NSMutableData> m_deliveryData;
    RetainPtr<NSURL> m_requestURL;
    RetainPtr<NSURL> m_responseURL;
    RetainPtr<NSString> m_mimeType;

    NPP m_plugin;
    uint16 m_transferMode;
    int32 m_offset;
    NPStream m_stream;
    RetainPtr<NSString> m_path;
    int m_fileDescriptor;
    BOOL m_sendNotification;
    void *m_notifyData;
    char *m_headers;
    RetainPtr<WebNetscapePluginView> m_pluginView;
    NPReason m_reason;
    bool m_isTerminated;
    bool m_newStreamSuccessful;
    
    WebCore::FrameLoader* m_frameLoader;
    RefPtr<WebCore::NetscapePlugInStreamLoader> m_loader;
    RetainPtr<NSMutableURLRequest> m_request;
    NPPluginFuncs *m_pluginFuncs;

    void deliverDataTimerFired(WebCore::Timer<WebNetscapePluginStream>* timer);
    WebCore::Timer<WebNetscapePluginStream> m_deliverDataTimer;
    
    WebNetscapePluginStream(WebCore::FrameLoader*);
    WebNetscapePluginStream(NSURLRequest *, NPP, bool sendNotification, void* notifyData);
};

#endif
