

#import "WebBaseNetscapePluginView.h"

#if ENABLE(NETSCAPE_PLUGIN_API)

@interface WebNetscapeContainerCheckContextInfo : NSObject {
    uint32 _checkRequestID;
    void (*_callback)(NPP npp, uint32, NPBool, void *);
    void *_context;
}

- (id)initWithCheckRequestID:(uint32)checkRequestID callbackFunc:(void (*)(NPP npp, uint32 checkID, NPBool allowed, void* context))callbackFunc context:(void*)context;
- (uint32)checkRequestID;
- (void (*)(NPP npp, uint32, NPBool, void*))callback;
- (void*)context;

@end

#endif // ENABLE(NETSCAPE_PLUGIN_API)
