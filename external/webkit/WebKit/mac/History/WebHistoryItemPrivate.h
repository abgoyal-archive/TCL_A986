

#import <WebKit/WebHistoryItem.h>

@interface WebHistoryItem (WebPrivate)

+ (void)_releaseAllPendingPageCaches;

- (id)initWithURL:(NSURL *)URL title:(NSString *)title;

- (NSURL *)URL;
- (int)visitCount;
- (BOOL)lastVisitWasFailure;
- (void)_setLastVisitWasFailure:(BOOL)failure;

- (BOOL)_lastVisitWasHTTPNonGet;

- (NSString *)RSSFeedReferrer;
- (void)setRSSFeedReferrer:(NSString *)referrer;
- (NSCalendarDate *)_lastVisitedDate;

- (NSArray *)_redirectURLs;

- (WebHistoryItem *)targetItem;
- (NSString *)target;
- (BOOL)isTargetItem;
- (NSArray *)children;
- (NSDictionary *)dictionaryRepresentation;

// This should not be called directly for WebHistoryItems that are already included
// in WebHistory. Use -[WebHistory setLastVisitedTimeInterval:forItem:] instead.
- (void)_setLastVisitedTimeInterval:(NSTimeInterval)time;
// Transient properties may be of any ObjC type.  They are intended to be used to store state per back/forward list entry.
// The properties will not be persisted; when the history item is removed, the properties will be lost.
- (id)_transientPropertyForKey:(NSString *)key;
- (void)_setTransientProperty:(id)property forKey:(NSString *)key;

- (size_t)_getDailyVisitCounts:(const int**)counts;
- (size_t)_getWeeklyVisitCounts:(const int**)counts;

@end
