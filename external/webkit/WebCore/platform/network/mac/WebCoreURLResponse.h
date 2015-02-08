

@interface NSURLResponse (WebCoreURLResponse)
-(void)adjustMIMETypeIfNecessary;
@end

@interface NSURLResponse (Details)
- (void)_setMIMEType:(NSString *)type;
@end
