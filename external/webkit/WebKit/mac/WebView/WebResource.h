

#import <Foundation/Foundation.h>

@class WebResourcePrivate;


@interface WebResource : NSObject <NSCoding, NSCopying>
{
@private
    WebResourcePrivate *_private;
}

- (id)initWithData:(NSData *)data URL:(NSURL *)URL MIMEType:(NSString *)MIMEType textEncodingName:(NSString *)textEncodingName frameName:(NSString *)frameName;

- (NSData *)data;

- (NSURL *)URL;

- (NSString *)MIMEType;

- (NSString *)textEncodingName;

- (NSString *)frameName;

@end
