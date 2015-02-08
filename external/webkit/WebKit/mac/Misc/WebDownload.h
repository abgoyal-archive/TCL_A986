

#import <Foundation/NSURLDownload.h>

@class WebDownloadInternal;



@interface WebDownload : NSURLDownload
{
@private
    WebDownloadInternal *_webInternal;
}

@end

@interface NSObject (WebDownloadDelegate)

- (NSWindow *)downloadWindowForAuthenticationSheet:(WebDownload *)download;

@end
