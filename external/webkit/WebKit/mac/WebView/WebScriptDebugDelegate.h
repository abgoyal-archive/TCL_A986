

#import <Foundation/Foundation.h>

#if MAC_OS_X_VERSION_MAX_ALLOWED <= MAC_OS_X_VERSION_10_4
#define WebNSUInteger unsigned int
#else
#define WebNSUInteger NSUInteger
#endif

#if defined(BUILDING_ON_TIGER) || defined(BUILDING_ON_LEOPARD)
typedef int WebSourceId;
#else
typedef intptr_t WebSourceId;
#endif

@class WebView;
@class WebFrame;
@class WebScriptCallFrame;
@class WebScriptCallFramePrivate;
@class WebScriptObject;

extern NSString * const WebScriptErrorDomain;
extern NSString * const WebScriptErrorDescriptionKey;
extern NSString * const WebScriptErrorLineNumberKey;

enum {
    WebScriptGeneralErrorCode = -100
};

// WebScriptDebugDelegate messages

@interface NSObject (WebScriptDebugDelegate)

// some source was parsed, establishing a "source ID" (>= 0) for future reference
// this delegate method is deprecated, please switch to the new version below
- (void)webView:(WebView *)webView       didParseSource:(NSString *)source
                                                fromURL:(NSString *)url
                                               sourceId:(WebSourceId)sid
                                            forWebFrame:(WebFrame *)webFrame;

// some source was parsed, establishing a "source ID" (>= 0) for future reference
- (void)webView:(WebView *)webView       didParseSource:(NSString *)source
                                         baseLineNumber:(WebNSUInteger)lineNumber
                                                fromURL:(NSURL *)url
                                               sourceId:(WebSourceId)sid
                                            forWebFrame:(WebFrame *)webFrame;

// some source failed to parse
- (void)webView:(WebView *)webView  failedToParseSource:(NSString *)source
                                         baseLineNumber:(WebNSUInteger)lineNumber
                                                fromURL:(NSURL *)url
                                              withError:(NSError *)error
                                            forWebFrame:(WebFrame *)webFrame;

// just entered a stack frame (i.e. called a function, or started global scope)
- (void)webView:(WebView *)webView    didEnterCallFrame:(WebScriptCallFrame *)frame
                                               sourceId:(WebSourceId)sid
                                                   line:(int)lineno
                                            forWebFrame:(WebFrame *)webFrame;

// about to execute some code
- (void)webView:(WebView *)webView willExecuteStatement:(WebScriptCallFrame *)frame
                                               sourceId:(WebSourceId)sid
                                                   line:(int)lineno
                                            forWebFrame:(WebFrame *)webFrame;

// about to leave a stack frame (i.e. return from a function)
- (void)webView:(WebView *)webView   willLeaveCallFrame:(WebScriptCallFrame *)frame
                                               sourceId:(WebSourceId)sid
                                                   line:(int)lineno
                                            forWebFrame:(WebFrame *)webFrame;

// exception is being thrown
- (void)webView:(WebView *)webView   exceptionWasRaised:(WebScriptCallFrame *)frame
                                               sourceId:(WebSourceId)sid
                                                   line:(int)lineno
                                            forWebFrame:(WebFrame *)webFrame;
@end



// WebScriptCallFrame interface
//
// These objects are passed as arguments to the debug delegate.

@interface WebScriptCallFrame : NSObject
{
@private
    WebScriptCallFramePrivate* _private;
    id                         _userInfo;
}

// associate user info with frame
- (void)setUserInfo:(id)userInfo;

// retrieve user info
- (id)userInfo;

// get next frame on call stack (or nil if this is already the "global" frame)
- (WebScriptCallFrame *)caller;

// get array of WebScriptObjects for each scope (innermost first, last is always global object)
- (NSArray *)scopeChain;

// get name of function (if available) or nil
- (NSString *)functionName;

// get pending exception (if any) or nil
- (id)exception;

// evaluate a script (as if by "eval") in the context of this frame
- (id)evaluateWebScript:(NSString *)script;

@end

#undef WebNSUInteger

