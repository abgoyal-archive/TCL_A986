

#import <Foundation/Foundation.h>

#if MAC_OS_X_VERSION_MAX_ALLOWED <= MAC_OS_X_VERSION_10_4
#define WebNSUInteger unsigned int
#else
#define WebNSUInteger NSUInteger
#endif

enum {
    WebCacheModelDocumentViewer = 0,
    WebCacheModelDocumentBrowser = 1,
    WebCacheModelPrimaryWebBrowser = 2
};
typedef WebNSUInteger WebCacheModel;

@class WebPreferencesPrivate;

extern NSString *WebPreferencesChangedNotification;

@interface WebPreferences: NSObject <NSCoding>
{
@private
    WebPreferencesPrivate *_private;
}

+ (WebPreferences *)standardPreferences;

- (id)initWithIdentifier:(NSString *)anIdentifier;

- (NSString *)identifier;

- (NSString *)standardFontFamily;

- (void)setStandardFontFamily:(NSString *)family;

- (NSString *)fixedFontFamily;

- (void)setFixedFontFamily:(NSString *)family;

- (NSString *)serifFontFamily;

- (void)setSerifFontFamily:(NSString *)family;

- (NSString *)sansSerifFontFamily;

- (void)setSansSerifFontFamily:(NSString *)family;

- (NSString *)cursiveFontFamily;

- (void)setCursiveFontFamily:(NSString *)family;

- (NSString *)fantasyFontFamily;

- (void)setFantasyFontFamily:(NSString *)family;

- (int)defaultFontSize;

- (void)setDefaultFontSize:(int)size;

- (int)defaultFixedFontSize;

- (void)setDefaultFixedFontSize:(int)size;

- (int)minimumFontSize;

- (void)setMinimumFontSize:(int)size;

- (int)minimumLogicalFontSize;

- (void)setMinimumLogicalFontSize:(int)size;

- (NSString *)defaultTextEncodingName;

- (void)setDefaultTextEncodingName:(NSString *)encoding;

- (BOOL)userStyleSheetEnabled;

- (void)setUserStyleSheetEnabled:(BOOL)flag;

- (NSURL *)userStyleSheetLocation;

- (void)setUserStyleSheetLocation:(NSURL *)URL;

- (BOOL)isJavaEnabled;

- (void)setJavaEnabled:(BOOL)flag;

- (BOOL)isJavaScriptEnabled;

- (void)setJavaScriptEnabled:(BOOL)flag;

- (BOOL)javaScriptCanOpenWindowsAutomatically;

- (void)setJavaScriptCanOpenWindowsAutomatically:(BOOL)flag;

- (BOOL)arePlugInsEnabled;

- (void)setPlugInsEnabled:(BOOL)flag;

- (BOOL)allowsAnimatedImages;

- (void)setAllowsAnimatedImages:(BOOL)flag;

- (BOOL)allowsAnimatedImageLooping;

- (void)setAllowsAnimatedImageLooping: (BOOL)flag;

- (void)setLoadsImagesAutomatically: (BOOL)flag;

- (BOOL)loadsImagesAutomatically;

- (void)setAutosaves:(BOOL)flag;

- (BOOL)autosaves;

- (void)setShouldPrintBackgrounds:(BOOL)flag;

- (BOOL)shouldPrintBackgrounds;

- (void)setPrivateBrowsingEnabled:(BOOL)flag;

- (BOOL)privateBrowsingEnabled;

- (void)setTabsToLinks:(BOOL)flag;

- (BOOL)tabsToLinks;

- (void)setUsesPageCache:(BOOL)usesPageCache;

- (BOOL)usesPageCache;

- (void)setCacheModel:(WebCacheModel)cacheModel;

- (WebCacheModel)cacheModel;

@end

#undef WebNSUInteger
