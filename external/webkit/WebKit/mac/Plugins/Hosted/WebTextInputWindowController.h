

#if USE(PLUGIN_HOST_PROCESS)

#ifndef WebTextInputWindowController_h
#define WebTextInputWindowController_h

@class WebTextInputPanel;

@interface WebTextInputWindowController : NSObject {
    WebTextInputPanel *_panel;
}

+ (WebTextInputWindowController *)sharedTextInputWindowController;

- (NSTextInputContext *)inputContext;
- (BOOL)interpretKeyEvent:(NSEvent *)event string:(NSString **)string;

@end

#endif // WebTextInputWindowController_h

#endif // USE(PLUGIN_HOST_PROCESS)
