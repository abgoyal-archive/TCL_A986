
#include "SDL_config.h"

/* Subclass of NSWindow to fix genie effect and support resize events  */
@interface SDL_QuartzWindow : NSWindow
- (void)miniaturize:(id)sender;
- (void)display;
- (void)setFrame:(NSRect)frameRect display:(BOOL)flag;
- (void)appDidHide:(NSNotification*)note;
- (void)appWillUnhide:(NSNotification*)note;
- (void)appDidUnhide:(NSNotification*)note;
- (id)initWithContentRect:(NSRect)contentRect styleMask:(unsigned int)styleMask backing:(NSBackingStoreType)backingType defer:(BOOL)flag;
@end

/* Delegate for our NSWindow to send SDLQuit() on close */
@interface SDL_QuartzWindowDelegate : NSObject
- (BOOL)windowShouldClose:(id)sender;
@end

/* Subclass of NSView to set cursor rectangle */
@interface SDL_QuartzView : NSView
- (void)resetCursorRects;
@end
