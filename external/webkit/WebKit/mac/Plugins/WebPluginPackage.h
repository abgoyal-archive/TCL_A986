

#import <Foundation/Foundation.h>

#import <WebKit/WebBasePluginPackage.h>

@protocol WebPluginViewFactory;

@interface WebPluginPackage : WebBasePluginPackage

- (Class)viewFactory;

@end
