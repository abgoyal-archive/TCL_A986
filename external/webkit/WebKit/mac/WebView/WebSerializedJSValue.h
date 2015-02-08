

@class WebSerializedJSValuePrivate;

typedef const struct OpaqueJSContext* JSContextRef;
typedef const struct OpaqueJSValue* JSValueRef;

@interface WebSerializedJSValue : NSObject {
@private
    WebSerializedJSValuePrivate *_private;
}

- (id)initWithValue:(JSValueRef)object context:(JSContextRef)sourceContext exception:(JSValueRef*)exception;
- (JSValueRef)deserialize:(JSContextRef)destinationContext;

@end
