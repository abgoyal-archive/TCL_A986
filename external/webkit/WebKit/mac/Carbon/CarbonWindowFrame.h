

#import <AppKit/AppKit.h>

@interface CarbonWindowFrame : NSView

// Instance variables.
{

    @private
    
    // Something we keep around just to return when it's asked for.
    unsigned int _styleMask;

}

@end // interface NSCarbonWindowFrame
