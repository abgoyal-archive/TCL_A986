

#import "WebDataSourcePrivate.h"
#import <wtf/Forward.h>

namespace WebCore {
    class DocumentLoader;
}

class WebDocumentLoaderMac;

@class DOMDocumentFragment;
@class DOMElement;
@class WebArchive;
@class WebResource;
@class WebView;

@interface WebDataSource (WebInternal)
- (void)_makeRepresentation;
- (BOOL)_isDocumentHTML;
- (WebView *)_webView;
- (NSURL *)_URL;
- (DOMElement *)_imageElementWithImageResource:(WebResource *)resource;
- (DOMDocumentFragment *)_documentFragmentWithImageResource:(WebResource *)resource;
- (DOMDocumentFragment *)_documentFragmentWithArchive:(WebArchive *)archive;
+ (NSMutableDictionary *)_repTypesAllowImageTypeOmission:(BOOL)allowImageTypeOmission;
- (void)_replaceSelectionWithArchive:(WebArchive *)archive selectReplacement:(BOOL)selectReplacement;
- (id)_initWithDocumentLoader:(PassRefPtr<WebDocumentLoaderMac>)loader;
- (void)_finishedLoading;
- (void)_receivedData:(NSData *)data;
- (void)_revertToProvisionalState;
- (void)_setMainDocumentError:(NSError *)error;
- (WebCore::DocumentLoader*)_documentLoader;
@end
