

#if ENABLE(NETSCAPE_PLUGIN_API)

@interface WebPluginRequest : NSObject
{
    NSURLRequest *_request;
    NSString *_frameName;
    void *_notifyData;
    BOOL _didStartFromUserGesture;
    BOOL _sendNotification;
}

- (id)initWithRequest:(NSURLRequest *)request frameName:(NSString *)frameName notifyData:(void *)notifyData sendNotification:(BOOL)sendNotification didStartFromUserGesture:(BOOL)currentEventIsUserGesture;

- (NSURLRequest *)request;
- (NSString *)frameName;
- (void *)notifyData;
- (BOOL)isCurrentEventUserGesture;
- (BOOL)sendNotification;

@end

#endif // ENABLE(NETSCAPE_PLUGIN_API)
