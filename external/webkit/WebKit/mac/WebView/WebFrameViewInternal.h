

#import <WebKit/WebFrameView.h>

@class WebDynamicScrollBarsView;
@class WebView;

@interface WebFrameView (WebInternal)

- (WebView *)_webView;
- (void)_setDocumentView:(NSView <WebDocumentView> *)view;
- (NSView <WebDocumentView> *)_makeDocumentViewForDataSource:(WebDataSource *)dataSource;
- (void)_setWebFrame:(WebFrame *)webFrame;
- (float)_verticalPageScrollDistance;
+ (NSMutableDictionary *)_viewTypesAllowImageTypeOmission:(BOOL)allowImageTypeOmission;
- (Class)_viewClassForMIMEType:(NSString *)MIMEType;
+ (Class)_viewClassForMIMEType:(NSString *)MIMEType allowingPlugins:(BOOL)allowPlugins;
+ (BOOL)_canShowMIMETypeAsHTML:(NSString *)MIMEType;
- (WebDynamicScrollBarsView *)_scrollView;
- (void)_install;

@end
