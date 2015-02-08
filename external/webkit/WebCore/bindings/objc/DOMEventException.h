

#import <JavaScriptCore/WebKitAvailability.h>

#if WEBKIT_VERSION_MAX_ALLOWED >= WEBKIT_VERSION_1_3

@class NSString;

extern NSString * const DOMEventException;

enum DOMEventExceptionCode {
    DOM_UNSPECIFIED_EVENT_TYPE_ERR = 0
};

#endif
