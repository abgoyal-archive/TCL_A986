

#import <AppKit/AppKit.h>
#import <WebKit/WebTypesInternal.h>

@interface DumpRenderTreePasteboard : NSPasteboard
- (NSInteger)declareType:(NSString *)type owner:(id)newOwner;
+ (void)releaseLocalPasteboards;
@end

