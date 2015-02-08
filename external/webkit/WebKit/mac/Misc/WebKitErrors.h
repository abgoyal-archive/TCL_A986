

extern NSString *WebKitErrorDomain;

extern NSString * const WebKitErrorMIMETypeKey;
extern NSString * const WebKitErrorPlugInNameKey;
extern NSString * const WebKitErrorPlugInPageURLStringKey;

enum {
    WebKitErrorCannotShowMIMEType =                             100,
    WebKitErrorCannotShowURL =                                  101,
    WebKitErrorFrameLoadInterruptedByPolicyChange =             102,
};

enum {
    WebKitErrorCannotFindPlugIn =                               200,
    WebKitErrorCannotLoadPlugIn =                               201,
    WebKitErrorJavaUnavailable =                                202,
};
