

#if ENABLE(NETSCAPE_PLUGIN_API)
#import <Cocoa/Cocoa.h>

#import "WebNetscapePluginPackage.h"
#import "WebPluginContainerCheck.h"
#import <wtf/OwnPtr.h>
#import <wtf/PassRefPtr.h>
#import <wtf/RefPtr.h>
#import <wtf/RetainPtr.h>

@class DOMElement;
@class WebDataSource;
@class WebFrame;
@class WebView;

namespace WebCore {
    class CString;
    class HTMLPlugInElement;
}

class WebHaltablePlugin;

@interface WebBaseNetscapePluginView : NSView
{
    RetainPtr<WebNetscapePluginPackage> _pluginPackage;
    
    WebFrame *_webFrame;
    
    int _mode;
    
    BOOL _triedAndFailedToCreatePlugin;
    BOOL _loadManually;
    BOOL _shouldFireTimers;
    BOOL _isStarted;
    BOOL _hasFocus;
    BOOL _isCompletelyObscured;
    BOOL _isPrivateBrowsingEnabled;
    BOOL _isHalted;
    BOOL _hasBeenHalted;
    
    RefPtr<WebCore::HTMLPlugInElement> _element;
    RetainPtr<NSString> _MIMEType;
    RetainPtr<NSURL> _baseURL;
    RetainPtr<NSURL> _sourceURL;
    
    OwnPtr<WebHaltablePlugin> _haltable;
    
    NSTrackingRectTag _trackingTag;
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

- (WebNetscapePluginPackage *)pluginPackage;

- (NSURL *)URLWithCString:(const char *)URLCString;
- (NSMutableURLRequest *)requestWithURLCString:(const char *)URLCString;

// Subclasses must override these.
- (void)handleMouseMoved:(NSEvent *)event;
- (void)setAttributeKeys:(NSArray *)keys andValues:(NSArray *)values;
- (void)focusChanged;

- (WebFrame *)webFrame;
- (WebDataSource *)dataSource;
- (WebView *)webView;
- (NSWindow *)currentWindow;
- (WebCore::HTMLPlugInElement*)element;

- (void)removeTrackingRect;
- (void)resetTrackingRect;

- (void)stopTimers;
- (void)startTimers;
- (void)restartTimers;

- (void)start;
- (void)stop;

- (void)halt;
- (void)resumeFromHalt;
- (BOOL)isHalted;
- (BOOL)hasBeenHalted;

- (void)addWindowObservers;
- (void)removeWindowObservers;
- (BOOL)shouldClipOutPlugin;

- (BOOL)convertFromX:(double)sourceX andY:(double)sourceY space:(NPCoordinateSpace)sourceSpace
                 toX:(double *)destX andY:(double *)destY space:(NPCoordinateSpace)destSpace;
- (WebCore::CString)resolvedURLStringForURL:(const char*)url target:(const char*)target;

- (void)invalidatePluginContentRect:(NSRect)rect;

@end


namespace WebKit {
#if !defined(BUILDING_ON_TIGER) && !defined(BUILDING_ON_LEOPARD)
WebCore::CString proxiesForURL(NSURL *);
#endif
    
bool getAuthenticationInfo(const char* protocolStr, const char* hostStr, int32_t port, const char* schemeStr, const char* realmStr,
                           WebCore::CString& username, WebCore::CString& password);
} 

#endif

