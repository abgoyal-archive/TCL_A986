

#import "WebIconDatabasePrivate.h"

namespace WebCore {
    class Image;
}

@interface WebIconDatabasePrivate : NSObject {
@public
    id delegate;
    BOOL delegateImplementsDefaultIconForURL;
    NSMutableDictionary *htmlIcons;
}
@end

@interface WebIconDatabase (WebInternal)
- (void)_sendNotificationForURL:(NSString *)URL;
- (void)_sendDidRemoveAllIconsNotification;
- (void)_shutDownIconDatabase;
- (void)_startUpIconDatabase;
@end

extern bool importToWebCoreFormat();
NSImage *webGetNSImage(WebCore::Image*, NSSize);
