

#import <Cocoa/Cocoa.h>

// Sent whenever a site icon has changed. The object of the notification is the icon database.
// The userInfo contains the site URL whose icon has changed.
// It can be accessed with the key WebIconNotificationUserInfoURLKey.
extern NSString *WebIconDatabaseDidAddIconNotification;

extern NSString *WebIconNotificationUserInfoURLKey;

extern NSString *WebIconDatabaseDirectoryDefaultsKey;
extern NSString *WebIconDatabaseEnabledDefaultsKey;

extern NSSize WebIconSmallSize;  // 16 x 16
extern NSSize WebIconMediumSize; // 32 x 32
extern NSSize WebIconLargeSize;  // 128 x 128

@class WebIconDatabasePrivate;

@interface WebIconDatabase : NSObject {

@private
    WebIconDatabasePrivate *_private;
    BOOL _isClosing;
}


+ (WebIconDatabase *)sharedIconDatabase;

- (NSImage *)iconForURL:(NSString *)URL withSize:(NSSize)size;

- (NSImage *)iconForURL:(NSString *)URL withSize:(NSSize)size cache:(BOOL)cache;

- (NSString *)iconURLForURL:(NSString *)URL;

- (NSImage *)defaultIconWithSize:(NSSize)size;
- (NSImage *)defaultIconForURL:(NSString *)URL withSize:(NSSize)size;

- (void)retainIconForURL:(NSString *)URL;

- (void)releaseIconForURL:(NSString *)URL;

+ (void)delayDatabaseCleanup;

+ (void)allowDatabaseCleanup;

- (void)setDelegate:(id)delegate;
- (id)delegate;

@end


