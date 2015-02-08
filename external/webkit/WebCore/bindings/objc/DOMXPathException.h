

#import <JavaScriptCore/WebKitAvailability.h>

#if WEBKIT_VERSION_MAX_ALLOWED >= WEBKIT_VERSION_3_0

@class NSString;

extern NSString * const DOMXPathException;

enum DOMXPathExceptionCode {
    DOM_INVALID_EXPRESSION_ERR      = 51,
    DOM_TYPE_ERR                    = 52
};

#endif
