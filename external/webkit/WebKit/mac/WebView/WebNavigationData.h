

#import <Foundation/Foundation.h>

@class WebFrame;
@class WebNavigationDataPrivate;
@class WebView;

@interface WebNavigationData : NSObject
{
@private
    WebNavigationDataPrivate *_private;
}

- (id)initWithURLString:(NSString *)url title:(NSString *)title originalRequest:(NSURLRequest *)request response:(NSURLResponse *)response hasSubstituteData:(BOOL)hasSubstituteData clientRedirectSource:(NSString *)redirectSource;
- (NSString *)url;
- (NSString *)title;
- (NSURLRequest *)originalRequest;
- (NSURLResponse *)response;
- (BOOL)hasSubstituteData;
- (NSString *)clientRedirectSource;

@end
