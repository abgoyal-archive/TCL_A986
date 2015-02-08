

#import <JavaVM/jni.h>


typedef enum {
    WebJNIReturnTypeInvalid = 0,
    WebJNIReturnTypeVoid,
    WebJNIReturnTypeObject,
    WebJNIReturnTypeBoolean,
    WebJNIReturnTypeByte,
    WebJNIReturnTypeChar,
    WebJNIReturnTypeShort,
    WebJNIReturnTypeInt,
    WebJNIReturnTypeLong,
    WebJNIReturnTypeFloat,
    WebJNIReturnTypeDouble
} WebJNIReturnType;

@interface NSObject (WebJavaPlugIn)

- (jobject)webPlugInGetApplet;

- (jvalue)webPlugInCallJava:(jobject)object
                   isStatic:(BOOL)isStatic
                 returnType:(WebJNIReturnType)returnType
                     method:(jmethodID)method
                  arguments:(jvalue*)args
                 callingURL:(NSURL *)url
       exceptionDescription:(NSString **)exceptionString;

@end
