

#if USE(PLUGIN_HOST_PROCESS)

#import "WebBaseNetscapePluginView.h"
#import "WebKitSystemInterface.h"

#import <wtf/RefPtr.h>

namespace WebKit {
    class HostedNetscapePluginStream;
    class NetscapePluginInstanceProxy;
}

@interface WebHostedNetscapePluginView : WebBaseNetscapePluginView<WebPluginManualLoader, WebPluginContainerCheckController>
{
    RetainPtr<NSArray> _attributeKeys;
    RetainPtr<NSArray> _attributeValues;
    
    RetainPtr<CALayer> _pluginLayer;
    WKSoftwareCARendererRef _softwareRenderer;
    
    NSSize _previousSize;
    RefPtr<WebKit::NetscapePluginInstanceProxy> _proxy;
    BOOL _pluginHostDied;
}

- (id)initWithFrame:(NSRect)r
      pluginPackage:(WebNetscapePluginPackage *)thePluginPackage
                URL:(NSURL *)URL
            baseURL:(NSURL *)baseURL
           MIMEType:(NSString *)MIME
      attributeKeys:(NSArray *)keys
    attributeValues:(NSArray *)values
       loadManually:(BOOL)loadManually
            element:(PassRefPtr<WebCore::HTMLPlugInElement>)element;

- (void)pluginHostDied;
- (CALayer *)pluginLayer;
- (void)webFrame:(WebFrame *)webFrame didFinishLoadWithReason:(NPReason)reason;

@end

#endif // USE(PLUGIN_HOST_PROCESS)

