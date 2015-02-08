

#import <WebKit/WebPreferences.h>
#import <Quartz/Quartz.h>

typedef enum {
    WebKitEditableLinkDefaultBehavior,
    WebKitEditableLinkAlwaysLive,
    WebKitEditableLinkOnlyLiveWithShiftKey,
    WebKitEditableLinkLiveWhenNotFocused,
    WebKitEditableLinkNeverLive
} WebKitEditableLinkBehavior;

typedef enum {
    WebTextDirectionSubmenuNeverIncluded,
    WebTextDirectionSubmenuAutomaticallyIncluded,
    WebTextDirectionSubmenuAlwaysIncluded
} WebTextDirectionSubmenuInclusionBehavior;

extern NSString *WebPreferencesChangedNotification;
extern NSString *WebPreferencesRemovedNotification;

@interface WebPreferences (WebPrivate)

// Preferences that might be public in a future release

- (BOOL)developerExtrasEnabled;
- (void)setDeveloperExtrasEnabled:(BOOL)flag;

- (BOOL)authorAndUserStylesEnabled;
- (void)setAuthorAndUserStylesEnabled:(BOOL)flag;

- (BOOL)applicationChromeModeEnabled;
- (void)setApplicationChromeModeEnabled:(BOOL)flag;

- (BOOL)usesEncodingDetector;
- (void)setUsesEncodingDetector:(BOOL)flag;

- (BOOL)respectStandardStyleKeyEquivalents;
- (void)setRespectStandardStyleKeyEquivalents:(BOOL)flag;

- (BOOL)showsURLsInToolTips;
- (void)setShowsURLsInToolTips:(BOOL)flag;

- (BOOL)textAreasAreResizable;
- (void)setTextAreasAreResizable:(BOOL)flag;

- (PDFDisplayMode)PDFDisplayMode;
- (void)setPDFDisplayMode:(PDFDisplayMode)mode;

- (BOOL)shrinksStandaloneImagesToFit;
- (void)setShrinksStandaloneImagesToFit:(BOOL)flag;

- (BOOL)automaticallyDetectsCacheModel;
- (void)setAutomaticallyDetectsCacheModel:(BOOL)automaticallyDetectsCacheModel;

- (BOOL)webArchiveDebugModeEnabled;
- (void)setWebArchiveDebugModeEnabled:(BOOL)webArchiveDebugModeEnabled;

- (BOOL)localFileContentSniffingEnabled;
- (void)setLocalFileContentSniffingEnabled:(BOOL)localFileContentSniffingEnabled;

- (BOOL)offlineWebApplicationCacheEnabled;
- (void)setOfflineWebApplicationCacheEnabled:(BOOL)offlineWebApplicationCacheEnabled;

- (BOOL)databasesEnabled;
- (void)setDatabasesEnabled:(BOOL)databasesEnabled;

- (BOOL)localStorageEnabled;
- (void)setLocalStorageEnabled:(BOOL)localStorageEnabled;

- (BOOL)isWebSecurityEnabled;
- (void)setWebSecurityEnabled:(BOOL)flag;

- (BOOL)allowUniversalAccessFromFileURLs;
- (void)setAllowUniversalAccessFromFileURLs:(BOOL)flag;

- (BOOL)zoomsTextOnly;
- (void)setZoomsTextOnly:(BOOL)zoomsTextOnly;

- (BOOL)isXSSAuditorEnabled;
- (void)setXSSAuditorEnabled:(BOOL)flag;

- (BOOL)experimentalNotificationsEnabled;
- (void)setExperimentalNotificationsEnabled:(BOOL)notificationsEnabled;

- (unsigned)pluginAllowedRunTime;
- (void)setPluginAllowedRunTime:(unsigned)allowedRunTime;

- (BOOL)isFrameSetFlatteningEnabled;
- (void)setFrameSetFlatteningEnabled:(BOOL)flag;

// zero means do AutoScale
- (float)PDFScaleFactor;
- (void)setPDFScaleFactor:(float)scale;

- (WebKitEditableLinkBehavior)editableLinkBehavior;
- (void)setEditableLinkBehavior:(WebKitEditableLinkBehavior)behavior;

- (WebTextDirectionSubmenuInclusionBehavior)textDirectionSubmenuInclusionBehavior;
- (void)setTextDirectionSubmenuInclusionBehavior:(WebTextDirectionSubmenuInclusionBehavior)behavior;

// Used to set preference specified in the test via LayoutTestController.overridePreference(..).
// For use with DumpRenderTree only.
- (void)_setPreferenceForTestWithValue:(NSString *)value forKey:(NSString *)key;

// If site-specific spoofing is enabled, some pages that do inappropriate user-agent string checks will be
// passed a nonstandard user-agent string to get them to work correctly. This method might be removed in
// the future when there's no more need for it.
- (BOOL)_useSiteSpecificSpoofing;
- (void)_setUseSiteSpecificSpoofing:(BOOL)newValue;

// WARNING: Allowing paste through the DOM API opens a security hole. We only use it for testing purposes.
- (BOOL)isDOMPasteAllowed;
- (void)setDOMPasteAllowed:(BOOL)DOMPasteAllowed;

- (NSString *)_ftpDirectoryTemplatePath;
- (void)_setFTPDirectoryTemplatePath:(NSString *)path;

- (void)_setForceFTPDirectoryListings:(BOOL)force;
- (BOOL)_forceFTPDirectoryListings;

- (NSString *)_localStorageDatabasePath;
- (void)_setLocalStorageDatabasePath:(NSString *)path;

- (BOOL)acceleratedCompositingEnabled;
- (void)setAcceleratedCompositingEnabled:(BOOL)enabled;

- (BOOL)showDebugBorders;
- (void)setShowDebugBorders:(BOOL)show;

- (BOOL)showRepaintCounter;
- (void)setShowRepaintCounter:(BOOL)show;

- (BOOL)webGLEnabled;
- (void)setWebGLEnabled:(BOOL)enabled;

- (BOOL)usesProxiedOpenPanel;
- (void)setUsesProxiedOpenPanel:(BOOL)enabled;

// Other private methods
- (void)_postPreferencesChangesNotification;
+ (WebPreferences *)_getInstanceForIdentifier:(NSString *)identifier;
+ (void)_setInstance:(WebPreferences *)instance forIdentifier:(NSString *)identifier;
+ (void)_removeReferenceForIdentifier:(NSString *)identifier;
- (NSTimeInterval)_backForwardCacheExpirationInterval;
+ (CFStringEncoding)_systemCFStringEncoding;
+ (void)_setInitialDefaultTextEncodingToSystemEncoding;
+ (void)_setIBCreatorID:(NSString *)string;

// For WebView's use only.
- (void)willAddToWebView;
- (void)didRemoveFromWebView;
@end
