

#import <Foundation/Foundation.h>

@class WebBasePluginPackage;
@class WebFrame;

@interface WebPluginDatabase : NSObject
{
    NSMutableDictionary *plugins;
    NSMutableSet *registeredMIMETypes;
    NSArray *plugInPaths;
    
    // Set of views with plugins attached
    NSMutableSet *pluginInstanceViews;
}

+ (WebPluginDatabase *)sharedDatabase;
+ (void)closeSharedDatabase; // avoids creating the database just to close it

// Plug-ins are returned in this order: New plug-in (WBPL), Mach-O Netscape, CFM Netscape
- (WebBasePluginPackage *)pluginForMIMEType:(NSString *)mimeType;
- (WebBasePluginPackage *)pluginForExtension:(NSString *)extension;

- (BOOL)isMIMETypeRegistered:(NSString *)MIMEType;

- (NSArray *)plugins;

- (void)refresh;

- (void)setPlugInPaths:(NSArray *)newPaths;

- (void)close;

- (void)addPluginInstanceView:(NSView *)view;
- (void)removePluginInstanceView:(NSView *)view;
- (void)removePluginInstanceViewsFor:(WebFrame *)webFrame;
- (void)destroyAllPluginInstanceViews;
@end

@interface NSObject (WebPlugInDatabase)

+ (void)setAdditionalWebPlugInPaths:(NSArray *)path;

@end
