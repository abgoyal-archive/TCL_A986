

#ifndef PluginStream_H
#define PluginStream_H

#include "CString.h"
#include "FileSystem.h"
#include "KURL.h"
#include "NetscapePlugInStreamLoader.h"
#include "PlatformString.h"
#include "PluginQuirkSet.h"
#include "ResourceRequest.h"
#include "ResourceResponse.h"
#include "StringHash.h"
#include "Timer.h"
#include "npruntime_internal.h"
#include <wtf/HashMap.h>
#include <wtf/OwnPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/Vector.h>

namespace WebCore {
    class Frame;
    class PluginStream;

    enum PluginStreamState { StreamBeforeStarted, StreamStarted, StreamStopped };

    class PluginStreamClient {
    public:
        virtual ~PluginStreamClient() {}
        virtual void streamDidFinishLoading(PluginStream*) {}
    };

    class PluginStream : public RefCounted<PluginStream>, private NetscapePlugInStreamLoaderClient {
    public:
        static PassRefPtr<PluginStream> create(PluginStreamClient* client, Frame* frame, const ResourceRequest& request, bool sendNotification, void* notifyData, const NPPluginFuncs* functions, NPP instance, const PluginQuirkSet& quirks)
        {
            return adoptRef(new PluginStream(client, frame, request, sendNotification, notifyData, functions, instance, quirks));
        }
        virtual ~PluginStream();
        
        void start();
        void stop();

        void startStream();

        void setLoadManually(bool loadManually) { m_loadManually = loadManually; }

        void sendJavaScriptStream(const KURL& requestURL, const CString& resultString);
        void cancelAndDestroyStream(NPReason);

        static NPP ownerForStream(NPStream*);

        // NetscapePlugInStreamLoaderClient
        virtual void didReceiveResponse(NetscapePlugInStreamLoader*, const ResourceResponse&);
        virtual void didReceiveData(NetscapePlugInStreamLoader*, const char*, int);
        virtual void didFail(NetscapePlugInStreamLoader*, const ResourceError&);
        virtual void didFinishLoading(NetscapePlugInStreamLoader*);
        virtual bool wantsAllStreams() const;

    private:
        PluginStream(PluginStreamClient*, Frame*, const ResourceRequest&, bool sendNotification, void* notifyData, const NPPluginFuncs*, NPP instance, const PluginQuirkSet&);

        void deliverData();
        void destroyStream(NPReason);
        void destroyStream();
#if PLATFORM(ANDROID)
        int deliveryDelay() const;
#endif
        ResourceRequest m_resourceRequest;
        ResourceResponse m_resourceResponse;

        PluginStreamClient* m_client;
        Frame* m_frame;
        RefPtr<NetscapePlugInStreamLoader> m_loader;
        void* m_notifyData;
        bool m_sendNotification;
        PluginStreamState m_streamState;
        bool m_loadManually;

        Timer<PluginStream> m_delayDeliveryTimer;
        void delayDeliveryTimerFired(Timer<PluginStream>*);

        OwnPtr< Vector<char> > m_deliveryData;

        PlatformFileHandle m_tempFileHandle;

        const NPPluginFuncs* m_pluginFuncs;
        NPP m_instance;
        uint16 m_transferMode;
        int32 m_offset;
        CString m_headers;
        CString m_path;
        NPReason m_reason;
        NPStream m_stream;
        PluginQuirkSet m_quirks;
    };

} // namespace WebCore

#endif
