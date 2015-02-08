

#import <Foundation/Foundation.h>

@class NSError;

@class WebHistoryItem;
@class WebHistoryPrivate;

extern NSString *WebHistoryItemsAddedNotification;
extern NSString *WebHistoryItemsRemovedNotification;
extern NSString *WebHistoryAllItemsRemovedNotification;
extern NSString *WebHistoryLoadedNotification;
extern NSString *WebHistorySavedNotification;

extern NSString *WebHistoryItemsKey;

@interface WebHistory : NSObject {
@private
    WebHistoryPrivate *_historyPrivate;
}

+ (WebHistory *)optionalSharedHistory;

+ (void)setOptionalSharedHistory:(WebHistory *)history;

- (BOOL)loadFromURL:(NSURL *)URL error:(NSError **)error;

- (BOOL)saveToURL:(NSURL *)URL error:(NSError **)error;

- (void)addItems:(NSArray *)newItems;

- (void)removeItems:(NSArray *)items;

- (void)removeAllItems;

- (NSArray *)orderedLastVisitedDays;

- (NSArray *)orderedItemsLastVisitedOnDay:(NSCalendarDate *)calendarDate;

- (WebHistoryItem *)itemForURL:(NSURL *)URL;

- (void)setHistoryItemLimit:(int)limit;

- (int)historyItemLimit;

- (void)setHistoryAgeInDaysLimit:(int)limit;

- (int)historyAgeInDaysLimit;

@end
