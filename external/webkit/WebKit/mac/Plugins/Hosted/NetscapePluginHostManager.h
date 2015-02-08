

#if USE(PLUGIN_HOST_PROCESS)

#ifndef NetscapePluginHostManager_h
#define NetscapePluginHostManager_h

#import <wtf/HashMap.h>
#import <wtf/PassRefPtr.h>

@class WebHostedNetscapePluginView;
@class WebNetscapePluginPackage;

namespace WebKit {

class NetscapePluginInstanceProxy;
class NetscapePluginHostProxy;

class NetscapePluginHostManager {
public:
    static NetscapePluginHostManager& shared();
    
    PassRefPtr<NetscapePluginInstanceProxy> instantiatePlugin(WebNetscapePluginPackage *, WebHostedNetscapePluginView *, NSString *mimeType, NSArray *attributeKeys, NSArray *attributeValues, NSString *userAgent, NSURL *sourceURL, bool fullFrame, bool isPrivateBrowsingEnabled, bool isAcceleratedCompositingEnabled);

    void pluginHostDied(NetscapePluginHostProxy*);

    static void createPropertyListFile(WebNetscapePluginPackage *);
    
    void didCreateWindow();
    
private:
    NetscapePluginHostProxy* hostForPackage(WebNetscapePluginPackage *, bool useProxiedOpenPanel);

    NetscapePluginHostManager();
    ~NetscapePluginHostManager();
    
    bool spawnPluginHost(WebNetscapePluginPackage *, mach_port_t clientPort, mach_port_t& pluginHostPort, ProcessSerialNumber& pluginHostPSN, bool useProxiedOpenPanel);
    
    bool initializeVendorPort();
    
    mach_port_t m_pluginVendorPort;
    
    // FIXME: This should really be a HashMap of RetainPtrs, but that doesn't work right now.
    typedef HashMap<WebNetscapePluginPackage*, NetscapePluginHostProxy*> PluginHostMap;
    PluginHostMap m_pluginHosts;
};
    
} // namespace WebKit

#endif // NetscapePluginHostManager_h
#endif // USE(PLUGIN_HOST_PROCESS)
