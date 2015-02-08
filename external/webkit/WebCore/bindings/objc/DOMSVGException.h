

#import <JavaScriptCore/WebKitAvailability.h>

#if WEBKIT_VERSION_MAX_ALLOWED >= WEBKIT_VERSION_LATEST

@class NSString;

extern NSString * const DOMSVGException;

enum DOMSVGException {
    DOM_SVG_WRONG_TYPE_ERR           = 0,
    DOM_SVG_INVALID_VALUE_ERR        = 1,
    DOM_SVG_MATRIX_NOT_INVERTABLE    = 2
};

#endif
