

#import <Cocoa/Cocoa.h>

@class WebIconFetcherPrivate;

@interface WebIconFetcher : NSObject {
    WebIconFetcherPrivate *_private;
}

- (void)cancel;

@end
