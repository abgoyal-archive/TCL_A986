

@class WebSecurityOriginPrivate;

@interface WebSecurityOrigin : NSObject {
    WebSecurityOriginPrivate *_private;
}

- (id)initWithURL:(NSURL *)url;

- (NSString*)protocol;
- (NSString*)host;

// Returns zero if the port is the default port for the protocol, non-zero otherwise
- (unsigned short)port;

// Returns the current total usage of all databases in this security origin in bytes
- (unsigned long long)usage;

- (unsigned long long)quota;
// Sets the storage quota (in bytes)
// If the quota is set to a value lower than the current usage, that quota will "stick" but no data will be purged to meet the new quota.  
// This will simply prevent new data from being added to databases in that origin
- (void)setQuota:(unsigned long long)quota;


@end
