

#ifndef WebSettings_h
#define WebSettings_h

#include "WebCommon.h"

namespace WebKit {

class WebString;
class WebURL;

// WebSettings is owned by the WebView and allows code to modify the settings for
// the WebView's page without any knowledge of WebCore itself.  For the most part,
// these functions have a 1:1 mapping with the methods in WebCore/page/settings.h.
class WebSettings {
public:
    virtual void setStandardFontFamily(const WebString&) = 0;
    virtual void setFixedFontFamily(const WebString&) = 0;
    virtual void setSerifFontFamily(const WebString&) = 0;
    virtual void setSansSerifFontFamily(const WebString&) = 0;
    virtual void setCursiveFontFamily(const WebString&) = 0;
    virtual void setFantasyFontFamily(const WebString&) = 0;
    virtual void setDefaultFontSize(int) = 0;
    virtual void setDefaultFixedFontSize(int) = 0;
    virtual void setMinimumFontSize(int) = 0;
    virtual void setMinimumLogicalFontSize(int) = 0;
    virtual void setDefaultTextEncodingName(const WebString&) = 0;
    virtual void setJavaScriptEnabled(bool) = 0;
    virtual void setWebSecurityEnabled(bool) = 0;
    virtual void setJavaScriptCanOpenWindowsAutomatically(bool) = 0;
    virtual void setLoadsImagesAutomatically(bool) = 0;
    virtual void setImagesEnabled(bool) = 0;
    virtual void setPluginsEnabled(bool) = 0;
    virtual void setDOMPasteAllowed(bool) = 0;
    virtual void setDeveloperExtrasEnabled(bool) = 0;
    virtual void setNeedsSiteSpecificQuirks(bool) = 0;
    virtual void setShrinksStandaloneImagesToFit(bool) = 0;
    virtual void setUsesEncodingDetector(bool) = 0;
    virtual void setTextAreasAreResizable(bool) = 0;
    virtual void setJavaEnabled(bool) = 0;
    virtual void setAllowScriptsToCloseWindows(bool) = 0;
    virtual void setUserStyleSheetLocation(const WebURL&) = 0;
    virtual void setUsesPageCache(bool) = 0;
    virtual void setDownloadableBinaryFontsEnabled(bool) = 0;
    virtual void setXSSAuditorEnabled(bool) = 0;
    virtual void setLocalStorageEnabled(bool) = 0;
    virtual void setEditableLinkBehaviorNeverLive() = 0;
    virtual void setFontRenderingModeNormal() = 0;
    virtual void setShouldPaintCustomScrollbars(bool) = 0;
    virtual void setDatabasesEnabled(bool) = 0;
    virtual void setAllowUniversalAccessFromFileURLs(bool) = 0;
    virtual void setTextDirectionSubmenuInclusionBehaviorNeverIncluded() = 0;
    virtual void setOfflineWebApplicationCacheEnabled(bool) = 0;
    virtual void setExperimentalWebGLEnabled(bool) = 0;
    virtual void setGeolocationEnabled(bool) = 0;

protected:
    ~WebSettings() { }
};

} // namespace WebKit

#endif
