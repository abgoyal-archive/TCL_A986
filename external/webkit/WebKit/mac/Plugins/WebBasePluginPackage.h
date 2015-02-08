

#import <WebCore/WebCoreViewFactory.h>

#if ENABLE(NETSCAPE_PLUGIN_API)
#import <WebKit/npfunctions.h>
#else
typedef void (*BP_CreatePluginMIMETypesPreferencesFuncPtr)(void);
#endif

@class WebPluginDatabase;

@protocol WebPluginManualLoader
- (void)pluginView:(NSView *)pluginView receivedResponse:(NSURLResponse *)response;
- (void)pluginView:(NSView *)pluginView receivedData:(NSData *)data;
- (void)pluginView:(NSView *)pluginView receivedError:(NSError *)error;
- (void)pluginViewFinishedLoading:(NSView *)pluginView;
@end

#define WebPluginExtensionsKey          @"WebPluginExtensions"
#define WebPluginDescriptionKey         @"WebPluginDescription"
#define WebPluginLocalizationNameKey    @"WebPluginLocalizationName"
#define WebPluginMIMETypesFilenameKey   @"WebPluginMIMETypesFilename"
#define WebPluginMIMETypesKey           @"WebPluginMIMETypes"
#define WebPluginNameKey                @"WebPluginName"
#define WebPluginTypeDescriptionKey     @"WebPluginTypeDescription"
#define WebPluginTypeEnabledKey         @"WebPluginTypeEnabled"

@interface WebBasePluginPackage : NSObject <WebCorePluginInfo>
{
    NSMutableSet *pluginDatabases;
    
    NSString *name;
    NSString *path;
    NSString *pluginDescription;

    NSBundle *bundle;
    CFBundleRef cfBundle;

    NSDictionary *MIMEToDescription;
    NSDictionary *MIMEToExtensions;
    NSMutableDictionary *extensionToMIME;
    
    BP_CreatePluginMIMETypesPreferencesFuncPtr BP_CreatePluginMIMETypesPreferences;
}

+ (WebBasePluginPackage *)pluginWithPath:(NSString *)pluginPath;
- (id)initWithPath:(NSString *)pluginPath;

- (BOOL)getPluginInfoFromPLists;

- (BOOL)load;
- (void)unload;

- (NSString *)name;
- (NSString *)path;
- (NSString *)filename;
- (NSString *)pluginDescription;
- (NSBundle *)bundle;

- (NSEnumerator *)extensionEnumerator;
- (NSEnumerator *)MIMETypeEnumerator;
- (NSString *)descriptionForMIMEType:(NSString *)MIMEType;
- (NSString *)MIMETypeForExtension:(NSString *)extension;
- (NSArray *)extensionsForMIMEType:(NSString *)MIMEType;

- (void)setName:(NSString *)theName;
- (void)setPath:(NSString *)thePath;
- (void)setPluginDescription:(NSString *)description;
- (void)setMIMEToDescriptionDictionary:(NSDictionary *)MIMEToDescriptionDictionary;
- (void)setMIMEToExtensionsDictionary:(NSDictionary *)MIMEToExtensionsDictionary;

- (BOOL)isQuickTimePlugIn;
- (BOOL)isJavaPlugIn;

- (BOOL)isNativeLibraryData:(NSData *)data;
- (UInt32)versionNumber;
- (void)wasAddedToPluginDatabase:(WebPluginDatabase *)database;
- (void)wasRemovedFromPluginDatabase:(WebPluginDatabase *)database;

@end
