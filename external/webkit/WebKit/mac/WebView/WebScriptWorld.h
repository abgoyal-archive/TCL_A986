

typedef struct OpaqueJSContext* JSGlobalContextRef;

@class WebScriptWorldPrivate;

@interface WebScriptWorld : NSObject {
@private
    WebScriptWorldPrivate *_private;
}

+ (WebScriptWorld *)standardWorld;
+ (WebScriptWorld *)world;

+ (WebScriptWorld *)scriptWorldForGlobalContext:(JSGlobalContextRef)globalContext;

@end
