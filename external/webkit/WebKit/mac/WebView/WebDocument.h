

#import <Cocoa/Cocoa.h>

@class NSError;
@class WebDataSource;

@protocol WebDocumentView <NSObject>

- (void)setDataSource:(WebDataSource *)dataSource;

- (void)dataSourceUpdated:(WebDataSource *)dataSource;

- (void)setNeedsLayout:(BOOL)flag;

- (void)layout;

- (void)viewWillMoveToHostWindow:(NSWindow *)hostWindow;

- (void)viewDidMoveToHostWindow;

@end


@protocol WebDocumentSearching <NSObject>
- (BOOL)searchFor:(NSString *)string direction:(BOOL)forward caseSensitive:(BOOL)caseFlag wrap:(BOOL)wrapFlag;
@end


@protocol WebDocumentText <NSObject>

- (BOOL)supportsTextEncoding;

- (NSString *)string;

- (NSAttributedString *)attributedString;

- (NSString *)selectedString;

- (NSAttributedString *)selectedAttributedString;


- (void)selectAll;

- (void)deselectAll;

@end


@protocol WebDocumentRepresentation <NSObject>
- (void)setDataSource:(WebDataSource *)dataSource;

- (void)receivedData:(NSData *)data withDataSource:(WebDataSource *)dataSource;

- (void)receivedError:(NSError *)error withDataSource:(WebDataSource *)dataSource;

- (void)finishedLoadingWithDataSource:(WebDataSource *)dataSource;

- (BOOL)canProvideDocumentSource;

- (NSString *)documentSource;

- (NSString *)title;

@end
