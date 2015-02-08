

// These are private because callers should be using the cover methods. They are in
// a Private (as opposed to Internal) header file because Safari uses some of them
// for managed preferences.
#define WebKitLogLevelPreferenceKey "WebKitLogLevel"
#define WebKitStandardFontPreferenceKey "WebKitStandardFont"
#define WebKitFixedFontPreferenceKey "WebKitFixedFont"
#define WebKitSerifFontPreferenceKey "WebKitSerifFont"
#define WebKitSansSerifFontPreferenceKey "WebKitSansSerifFont"
#define WebKitCursiveFontPreferenceKey "WebKitCursiveFont"
#define WebKitFantasyFontPreferenceKey "WebKitFantasyFont"
#define WebKitMinimumFontSizePreferenceKey "WebKitMinimumFontSize"
#define WebKitMinimumLogicalFontSizePreferenceKey "WebKitMinimumLogicalFontSize"
#define WebKitDefaultFontSizePreferenceKey "WebKitDefaultFontSize"
#define WebKitDefaultFixedFontSizePreferenceKey "WebKitDefaultFixedFontSize"
#define WebKitDefaultTextEncodingNamePreferenceKey "WebKitDefaultTextEncodingName"
#define WebKitUserStyleSheetEnabledPreferenceKey "WebKitUserStyleSheetEnabledPreferenceKey"
#define WebKitUserStyleSheetLocationPreferenceKey "WebKitUserStyleSheetLocationPreferenceKey"
#define WebKitShouldPrintBackgroundsPreferenceKey "WebKitShouldPrintBackgroundsPreferenceKey"
#define WebKitTextAreasAreResizablePreferenceKey "WebKitTextAreasAreResizable"
#define WebKitJavaEnabledPreferenceKey "WebKitJavaEnabled"
#define WebKitJavaScriptEnabledPreferenceKey "WebKitJavaScriptEnabled"
#define WebKitWebSecurityEnabledPreferenceKey "WebKitWebSecurityEnabled"
#define WebKitAllowUniversalAccessFromFileURLsPreferenceKey "WebKitAllowUniversalAccessFromFileURLs"
#define WebKitJavaScriptCanOpenWindowsAutomaticallyPreferenceKey "WebKitJavaScriptCanOpenWindowsAutomatically"
#define WebKitPluginsEnabledPreferenceKey "WebKitPluginsEnabled"
#define WebKitDatabasesEnabledPreferenceKey "WebKitDatabasesEnabled"
#define WebKitLocalStorageEnabledPreferenceKey "WebKitLocalStorageEnabled"
#define WebKitExperimentalNotificationsEnabledPreferenceKey "WebKitExperimentalNotificationsEnabled"
#define WebKitAllowAnimatedImagesPreferenceKey "WebKitAllowAnimatedImagesPreferenceKey"
#define WebKitAllowAnimatedImageLoopingPreferenceKey "WebKitAllowAnimatedImageLoopingPreferenceKey"
#define WebKitDisplayImagesKey "WebKitDisplayImagesKey"
#define WebKitBackForwardCacheExpirationIntervalKey "WebKitBackForwardCacheExpirationIntervalKey"
#define WebKitTabToLinksPreferenceKey "WebKitTabToLinksPreferenceKey"
#define WebKitPrivateBrowsingEnabledPreferenceKey "WebKitPrivateBrowsingEnabled"
#define WebKitIconDatabaseLocationKey "WebKitIconDatabaseLocation"
#define WebKitIconDatabaseEnabledPreferenceKey "WebKitIconDatabaseEnabled"
#define WebKitUsesPageCachePreferenceKey "WebKitUsesPageCachePreferenceKey"
#define WebKitCacheModelPreferenceKey "WebKitCacheModelPreferenceKey"
#define WebKitLocalStorageDatabasePathPreferenceKey "WebKitLocalStorageDatabasePath"

// These are private both because callers should be using the cover methods and because the
// cover methods themselves are private.
#define WebKitRespectStandardStyleKeyEquivalentsPreferenceKey "WebKitRespectStandardStyleKeyEquivalents"
#define WebKitShowsURLsInToolTipsPreferenceKey "WebKitShowsURLsInToolTips"
#define WebKitPDFDisplayModePreferenceKey "WebKitPDFDisplayMode"
#define WebKitPDFScaleFactorPreferenceKey "WebKitPDFScaleFactor"
#define WebKitEditableLinkBehaviorPreferenceKey "WebKitEditableLinkBehavior"

// Window display is throttled to 60 frames per second if WebKitThrottleWindowDisplayPreferenceKey
// is set to YES.  The window display throttle is OFF by default for compatibility with Mac OS X
// 10.4.6.
#define WebKitThrottleWindowDisplayPreferenceKey "WebKitThrottleWindowDisplay"

// CoreGraphics deferred updates are disabled if WebKitEnableCoalescedUpdatesPreferenceKey is set
// to NO, or has no value.  For compatibility with Mac OS X 10.4.6, deferred updates are OFF by
// default.
#define WebKitEnableDeferredUpdatesPreferenceKey "WebKitEnableDeferredUpdates"

// For debugging only.  Don't use these.
#define WebKitPageCacheSizePreferenceKey "WebKitPageCacheSizePreferenceKey"
#define WebKitObjectCacheSizePreferenceKey "WebKitObjectCacheSizePreferenceKey"

// From WebHistory.h
#define WebKitHistoryItemLimitKey "WebKitHistoryItemLimit" // default: "1000"
#define WebKitHistoryAgeInDaysLimitKey "WebKitHistoryAgeInDaysLimit" // default: "7"

// Windows-specific keys
#define WebKitFontSmoothingTypePreferenceKey "WebKitFontSmoothingType" // default: FontSmoothingTypeMedium (2)
#define WebKitFontSmoothingContrastPreferenceKey "WebKitFontSmoothingContrast" // default: "2"
#define WebKitCookieStorageAcceptPolicyPreferenceKey "WebKitCookieStorageAcceptPolicy" // default: WebKitCookieStorageAcceptPolicyOnlyFromMainDocumentDomain

#define WebContinuousSpellCheckingEnabledPreferenceKey "WebContinuousSpellCheckingEnabled" // default: false
#define WebGrammarCheckingEnabledPreferenceKey "WebGrammarCheckingEnabled" // default: false

#define AllowContinuousSpellCheckingPreferenceKey "AllowContinuousSpellCheckingPreferenceKey" // default: true

#define WebKitDOMPasteAllowedPreferenceKey "WebKitDOMPasteAllowedPreferenceKey" // default: false

#define WebKitApplicationChromeModePreferenceKey "WebKitApplicationChromeMode" // default: false

#define WebKitOfflineWebApplicationCacheEnabledPreferenceKey "WebKitOfflineWebApplicationCacheEnabled" // default: false

// If this key is present and has a value of true, we have already removed the default values from the user's preferences <rdar://problem/5214504>
#define WebKitDidMigrateDefaultSettingsFromSafari3BetaPreferenceKey "WebKitDidMigrateDefaultSettingsFromSafari3BetaPreferenceKey"

#define WebKitDidMigrateWebKitPreferencesToCFPreferencesPreferenceKey "WebKitDidMigrateWebKitPreferencesToCFPreferences"

#define WebKitDeveloperExtrasEnabledPreferenceKey "WebKitDeveloperExtras"
#define DisableWebKitDeveloperExtrasPreferenceKey "DisableWebKitDeveloperExtras"

#define WebKitAuthorAndUserStylesEnabledPreferenceKey "WebKitAuthorAndUserStylesEnabled"

#define WebKitPaintCustomScrollbarsPreferenceKey "WebKitPaintCustomScrollbars"

#define WebKitPaintNativeControlsPreferenceKey "WebKitPaintNativeControls"

#define WebKitZoomsTextOnlyPreferenceKey "WebKitZoomsTextOnly"

#define WebKitXSSAuditorEnabledPreferenceKey "WebKitXSSAuditorEnabled"

#define WebKitUseHighResolutionTimersPreferenceKey "WebKitUseHighResolutionTimers"

#define WebKitPluginAllowedRunTimePreferenceKey "WebKitPluginAllowedRunTime"

#define WebKitFrameSetFlatteningEnabledPreferenceKey "WebKitFrameSetFlatteningEnabled"

#define WebKitAcceleratedCompositingEnabledPreferenceKey "WebKitAcceleratedCompositingEnabled"

#define WebKitCustomDragCursorsEnabledPreferenceKey "WebKitCustomDragCursorsEnabled"
