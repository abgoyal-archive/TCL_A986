

#import <Foundation/Foundation.h>

@class WebArchivePrivate;
@class WebResource;

extern NSString *WebArchivePboardType;

@interface WebArchive : NSObject <NSCoding, NSCopying>
{
    @private
    WebArchivePrivate *_private;
}

- (id)initWithMainResource:(WebResource *)mainResource subresources:(NSArray *)subresources subframeArchives:(NSArray *)subframeArchives;

- (id)initWithData:(NSData *)data;

- (WebResource *)mainResource;

- (NSArray *)subresources;

- (NSArray *)subframeArchives;

- (NSData *)data;

@end
