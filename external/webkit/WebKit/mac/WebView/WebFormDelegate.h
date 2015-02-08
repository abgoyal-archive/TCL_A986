

#import <AppKit/AppKit.h>

@class DOMElement;
@class DOMHTMLInputElement;
@class DOMHTMLTextAreaElement;
@class WebFrame;

@protocol WebFormSubmissionListener <NSObject>
- (void)continue;
@end

@protocol WebFormDelegate <NSObject>

// Various methods send by controls that edit text to their delegates, which are all
// analogous to similar methods in AppKit/NSControl.h.
// These methods are forwarded from widgets used in forms to the WebFormDelegate.

- (void)textFieldDidBeginEditing:(DOMHTMLInputElement *)element inFrame:(WebFrame *)frame;
- (void)textFieldDidEndEditing:(DOMHTMLInputElement *)element inFrame:(WebFrame *)frame;
- (void)textDidChangeInTextField:(DOMHTMLInputElement *)element inFrame:(WebFrame *)frame;
- (void)textDidChangeInTextArea:(DOMHTMLTextAreaElement *)element inFrame:(WebFrame *)frame;

- (BOOL)textField:(DOMHTMLInputElement *)element doCommandBySelector:(SEL)commandSelector inFrame:(WebFrame *)frame;
- (BOOL)textField:(DOMHTMLInputElement *)element shouldHandleEvent:(NSEvent *)event inFrame:(WebFrame *)frame;

// Sent when a form is just about to be submitted (before the load is started)
// listener must be sent continue when the delegate is done.
- (void)frame:(WebFrame *)frame sourceFrame:(WebFrame *)sourceFrame willSubmitForm:(DOMElement *)form
    withValues:(NSDictionary *)values submissionListener:(id <WebFormSubmissionListener>)listener;

@end

@interface WebFormDelegate : NSObject <WebFormDelegate>
@end
