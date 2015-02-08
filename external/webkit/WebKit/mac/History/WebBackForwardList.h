

#import <Foundation/Foundation.h>

#if MAC_OS_X_VERSION_MAX_ALLOWED <= MAC_OS_X_VERSION_10_4
#define WebNSUInteger unsigned int
#else
#define WebNSUInteger NSUInteger
#endif

@class WebHistoryItem;
@class WebBackForwardListPrivate;

@interface WebBackForwardList : NSObject {
@private
    WebBackForwardListPrivate *_private;
}

- (void)addItem:(WebHistoryItem *)item;

- (void)goBack;

- (void)goForward;

- (void)goToItem:(WebHistoryItem *)item;

- (WebHistoryItem *)backItem;

- (WebHistoryItem *)currentItem;

- (WebHistoryItem *)forwardItem;

- (NSArray *)backListWithLimit:(int)limit;

- (NSArray *)forwardListWithLimit:(int)limit;

- (int)capacity;

- (void)setCapacity:(int)size;

- (int)backListCount;

- (int)forwardListCount;

- (BOOL)containsItem:(WebHistoryItem *)item;

- (WebHistoryItem *)itemAtIndex:(int)index;

@end

@interface WebBackForwardList(WebBackForwardListDeprecated)

// The following methods are deprecated, and exist for backward compatibility only.
// Use -[WebPreferences setUsesPageCache] and -[WebPreferences usesPageCache]
// instead.

- (void)setPageCacheSize:(WebNSUInteger)size;

- (WebNSUInteger)pageCacheSize;
@end

#undef WebNSUInteger
