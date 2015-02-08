

#import <Cocoa/Cocoa.h>
#import <WebKit/npfunctions.h>

@interface MenuHandler : NSObject {
    NSMenu *menu;
    NPNetscapeFuncs *browserFuncs;
    NPP instance;
}

- (id)initWithBrowserFuncs:(NPNetscapeFuncs *)theBrowserFuncs instance:(NPP)theInstance;

- (NSMenu *)menu;

@end
