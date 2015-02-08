

#if ENABLE(DATABASE)
 
extern NSString *WebDatabaseDirectoryDefaultsKey;

extern NSString *WebDatabaseDisplayNameKey;
extern NSString *WebDatabaseExpectedSizeKey;
extern NSString *WebDatabaseUsageKey;

// Posted with an origin is created from scratch, gets a new database, has a database deleted, has a quota change, etc
// The notification object will be a WebSecurityOrigin object corresponding to the origin.
extern NSString *WebDatabaseDidModifyOriginNotification;

// Posted when a database is created, its size increases, its display name changes, or its estimated size changes, or the database is removed
// The notification object will be a WebSecurityOrigin object corresponding to the origin.
// The notification userInfo will have a WebDatabaseNameKey whose value is the database name.
extern NSString *WebDatabaseDidModifyDatabaseNotification;
extern NSString *WebDatabaseIdentifierKey;

@class WebSecurityOrigin;

@interface WebDatabaseManager : NSObject

+ (WebDatabaseManager *)sharedWebDatabaseManager;

// Will return an array of WebSecurityOrigin objects.
- (NSArray *)origins;

// Will return an array of strings, the identifiers of each database in the given origin.
- (NSArray *)databasesWithOrigin:(WebSecurityOrigin *)origin;

// Will return the dictionary describing everything about the database for the passed identifier and origin.
- (NSDictionary *)detailsForDatabase:(NSString *)databaseIdentifier withOrigin:(WebSecurityOrigin *)origin;

- (void)deleteAllDatabases; // Deletes all databases and all origins.
- (void)deleteOrigin:(WebSecurityOrigin *)origin;
- (void)deleteDatabase:(NSString *)databaseIdentifier withOrigin:(WebSecurityOrigin *)origin;

@end

#endif
