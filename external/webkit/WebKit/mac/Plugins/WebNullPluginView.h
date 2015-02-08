

#import <AppKit/AppKit.h>

@class DOMElement;

@interface WebNullPluginView : NSImageView
{
    NSError *error;
    DOMElement *element;
}

- (id)initWithFrame:(NSRect)frame error:(NSError *)error DOMElement:(DOMElement *)element;

@end
