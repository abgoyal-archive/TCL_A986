

#import <AppKit/AppKit.h>

@interface WebClipView : NSClipView
{
    BOOL _haveAdditionalClip;
    NSRect _additionalClip;
}

- (void)setAdditionalClip:(NSRect)additionalClip;
- (void)resetAdditionalClip;
- (BOOL)hasAdditionalClip;
- (NSRect)additionalClip;

@end
