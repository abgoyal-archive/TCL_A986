

#import <Cocoa/Cocoa.h>

@interface WebJavaScriptTextInputPanel : NSWindowController
{
    IBOutlet NSTextField *prompt;
    IBOutlet NSTextField *textInput;
}

- (id)initWithPrompt:(NSString *)prompt text:(NSString *)text;
- (NSString *)text;

- (IBAction)pressedCancel:(id)sender;
- (IBAction)pressedOK:(id)sender;

@end
