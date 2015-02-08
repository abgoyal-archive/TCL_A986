

#import <WebKit/WebHistory.h>

// FIXME: This notification should become public API.
extern NSString *WebHistoryItemsDiscardedWhileLoadingNotification;

@interface WebHistory (WebPrivate)

// FIXME: The following SPI is used by Safari. Should it be made into public API?
- (WebHistoryItem *)_itemForURLString:(NSString *)URLString;

- (NSArray *)allItems;

- (NSData *)_data;

+ (void)_setVisitedLinkTrackingEnabled:(BOOL)visitedLinkTrackingEnabled;
+ (void)_removeAllVisitedLinks;
@end
