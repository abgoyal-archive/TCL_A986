

#import <Cocoa/Cocoa.h>

#if !defined(ENABLE_PLUGIN_PROXY_FOR_VIDEO)
#define ENABLE_PLUGIN_PROXY_FOR_VIDEO 1
#endif

#if ENABLE_PLUGIN_PROXY_FOR_VIDEO
@class WebMediaPlayerProxy;
#endif

@interface NSObject (WebPlugInContainerPrivate)

- (id)_webPluginContainerCheckIfAllowedToLoadRequest:(NSURLRequest *)Request inFrame:(NSString *)target resultObject:(id)obj selector:(SEL)selector;

- (void)_webPluginContainerCancelCheckIfAllowedToLoadRequest:(id)checkIdentifier;

#if ENABLE_PLUGIN_PROXY_FOR_VIDEO
- (void)_webPluginContainerSetMediaPlayerProxy:(WebMediaPlayerProxy *)proxy forElement:(DOMElement *)element;

- (void)_webPluginContainerPostMediaPlayerNotification:(int)notification forElement:(DOMElement *)element;
#endif

@end
