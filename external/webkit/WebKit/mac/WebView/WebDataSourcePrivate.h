

#import <WebKit/WebDataSource.h>

@interface WebDataSource (WebPrivate)

- (NSFileWrapper *)_fileWrapperForURL:(NSURL *)URL;
- (void)_addSubframeArchives:(NSArray *) archives;
- (NSError *)_mainDocumentError;
- (NSString *)_responseMIMEType;

- (BOOL)_transferApplicationCache:(NSString*)destinationBundleIdentifier;

@end
