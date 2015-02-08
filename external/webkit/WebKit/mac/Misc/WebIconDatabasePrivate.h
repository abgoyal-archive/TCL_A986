

#import <WebKit/WebIconDatabase.h>

// FIXME: Some of the following is not API and should be moved
// either inside WebIconDatabase.mm, or to WebIconDatabaseInternal.h.

// Sent when all icons are removed from the database. The object of the notification is 
// the icon database. There is no userInfo. Clients should react by removing any cached
// icon images from the user interface. Clients need not and should not call 
// releaseIconForURL: in response to this notification.
extern NSString *WebIconDatabaseDidRemoveAllIconsNotification;

// Key to store the path to look for old style icons in to convert to the new icon db
extern NSString *WebIconDatabaseImportDirectoryDefaultsKey;

@interface WebIconDatabase (WebPendingPublic)

- (BOOL)isEnabled;

- (void)setEnabled:(BOOL)flag;

- (void)removeAllIcons;

@end

@interface WebIconDatabase (WebPrivate)

+ (void)_checkIntegrityBeforeOpening;

@end

