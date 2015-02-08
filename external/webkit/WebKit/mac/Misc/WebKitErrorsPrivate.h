

#import <WebKit/WebKitErrors.h>

#define WebKitErrorPlugInCancelledConnection 203
// FIXME: WebKitErrorPlugInWillHandleLoad is used for the cancel we do to prevent loading plugin content twice.  See <rdar://problem/4258008>
#define WebKitErrorPlugInWillHandleLoad 204

enum {
    WebKitErrorCannotUseRestrictedPort =                        103,
};

enum {
    WebKitErrorGeolocationLocationUnknown  =                    300,
};

@interface NSError (WebKitExtras)
+ (NSError *)_webKitErrorWithCode:(int)code failingURL:(NSString *)URL;
+ (NSError *)_webKitErrorWithDomain:(NSString *)domain code:(int)code URL:(NSURL *)URL;

- (id)_initWithPluginErrorCode:(int)code
                    contentURL:(NSURL *)contentURL
                 pluginPageURL:(NSURL *)pluginPageURL
                    pluginName:(NSString *)pluginName
                      MIMEType:(NSString *)MIMEType;
@end
