

#import <JavaScriptCore/WebKitAvailability.h>

#if WEBKIT_VERSION_MAX_ALLOWED >= WEBKIT_VERSION_1_3

@class NSString;

extern NSString * const DOMRangeException;

enum DOMRangeExceptionCode {
    DOM_BAD_BOUNDARYPOINTS_ERR        = 1,
    DOM_INVALID_NODE_TYPE_ERR         = 2
};

#endif
