

#import <WebKit/WebResource.h>

@interface WebResource (WebResourcePrivate)

- (id)_initWithData:(NSData *)data 
                URL:(NSURL *)URL 
           MIMEType:(NSString *)MIMEType 
   textEncodingName:(NSString *)textEncodingName 
          frameName:(NSString *)frameName 
           response:(NSURLResponse *)response
           copyData:(BOOL)copyData;

- (id)_initWithData:(NSData *)data URL:(NSURL *)URL response:(NSURLResponse *)response;

- (void)_ignoreWhenUnarchiving;

- (NSFileWrapper *)_fileWrapperRepresentation;
- (NSURLResponse *)_response;
- (NSString *)_stringValue;

@end
