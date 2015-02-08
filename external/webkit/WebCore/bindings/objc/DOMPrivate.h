

#import <WebCore/DOM.h>

@interface DOMNode (DOMNodeExtensionsPendingPublic)
- (NSImage *)renderedImage;
- (NSArray *)textRects;
@end

// FIXME: this should be removed as soon as all internal Apple uses of it have been replaced with
// calls to the public method - (NSColor *)color.
@interface DOMRGBColor (WebPrivate)
- (NSColor *)_color;
@end

// FIXME: this should be removed as soon as all internal Apple uses of it have been replaced with
// calls to the public method - (NSString *)text.
@interface DOMRange (WebPrivate)
- (NSString *)_text;
@end

@interface DOMRange (DOMRangeExtensions)
- (NSRect)boundingBox;
- (NSArray *)lineBoxRects; // Deprecated. Use textRects instead.
- (NSArray *)textRects;
@end

@interface DOMElement (WebPrivate)
- (NSFont *)_font;
- (NSData *)_imageTIFFRepresentation;
- (NSURL *)_getURLAttribute:(NSString *)name;
- (BOOL)isFocused;
@end

@interface DOMCSSStyleDeclaration (WebPrivate)
- (NSString *)_fontSizeDelta;
- (void)_setFontSizeDelta:(NSString *)fontSizeDelta;
@end

@interface DOMHTMLDocument (WebPrivate)
- (DOMDocumentFragment *)_createDocumentFragmentWithMarkupString:(NSString *)markupString baseURLString:(NSString *)baseURLString;
- (DOMDocumentFragment *)_createDocumentFragmentWithText:(NSString *)text;
@end

// All the methods in this category are used by Safari forms autofill and should not be used for any other purpose.
// Each one should eventually be replaced by public DOM API, and when that happens Safari will switch to implementations 
// using that public API, and these will be deleted.
@interface DOMHTMLInputElement (FormsAutoFillTransition)
- (BOOL)_isAutofilled;
- (BOOL)_isTextField;
- (NSRect)_rectOnScreen; // bounding box of the text field, in screen coordinates
- (void)_replaceCharactersInRange:(NSRange)targetRange withString:(NSString *)replacementString selectingFromIndex:(int)index;
- (NSRange)_selectedRange;
- (void)_setAutofilled:(BOOL)filled;
@end

// These changes are necessary to detect whether a form input was modified by a user
// or javascript
@interface DOMHTMLInputElement (FormPromptAdditions)
- (BOOL)_isEdited;
@end

@interface DOMHTMLTextAreaElement (FormPromptAdditions)
- (BOOL)_isEdited;
@end

// All the methods in this category are used by Safari forms autofill and should not be used for any other purpose.
// They are stopgap measures until we finish transitioning form controls to not use NSView. Each one should become
// replaceable by public DOM API, and when that happens Safari will switch to implementations using that public API,
// and these will be deleted.
@interface DOMHTMLSelectElement (FormsAutoFillTransition)
- (void)_activateItemAtIndex:(int)index;
@end
