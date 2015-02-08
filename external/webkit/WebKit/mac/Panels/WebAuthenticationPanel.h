

#import <Cocoa/Cocoa.h>
#import <Foundation/NSURLCredentialStorage.h>

@class NSURLAuthenticationChallenge;

@interface WebAuthenticationPanel : NSObject
{
    IBOutlet id mainLabel;
    IBOutlet id panel;
    IBOutlet id password;
    IBOutlet id smallLabel;
    IBOutlet id username;
    IBOutlet id imageView;
    IBOutlet id remember;
    IBOutlet NSTextField *separateRealmLabel;
    BOOL nibLoaded;
    BOOL usingSheet;
    id callback;
    SEL selector;
    NSURLAuthenticationChallenge *challenge;
}

-(id)initWithCallback:(id)cb selector:(SEL)sel;

// Interface-related methods
- (IBAction)cancel:(id)sender;
- (IBAction)logIn:(id)sender;

- (BOOL)loadNib;

- (void)runAsModalDialogWithChallenge:(NSURLAuthenticationChallenge *)chall;
- (void)runAsSheetOnWindow:(NSWindow *)window withChallenge:(NSURLAuthenticationChallenge *)chall;

- (void)sheetDidEnd:(NSWindow *)sheet returnCode:(int)returnCode contextInfo:(void  *)contextInfo;

@end

// This is in the header so it can be used from the nib file
@interface NonBlockingPanel : NSPanel
@end

