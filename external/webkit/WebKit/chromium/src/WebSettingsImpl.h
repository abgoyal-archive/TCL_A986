

#ifndef WebSettingsImpl_h
#define WebSettingsImpl_h

// TODO(jorlow): Remove this hack once WebView is free of glue.
#include "../public/WebSettings.h"

namespace WebCore {
class Settings;
}

namespace WebKit {

class WebSettingsImpl : public WebSettings {
public:
    explicit WebSettingsImpl(WebCore::Settings*);
    virtual ~WebSettingsImpl() { }

    virtual void setStandardFontFamily(const WebString&);
    virtual void setFixedFontFamily(const WebString&);
    virtual void setSerifFontFamily(const WebString&);
    virtual void setSansSerifFontFamily(const WebString&);
    virtual void setCursiveFontFamily(const WebString&);
    virtual void setFantasyFontFamily(const WebString&);
    virtual void setDefaultFontSize(int);
    virtual void setDefaultFixedFontSize(int);
    virtual void setMinimumFontSize(int);
    virtual void setMinimumLogicalFontSize(int);
    virtual void setDefaultTextEncodingName(const WebString&);
    virtual void setJavaScriptEnabled(bool);
    virtual void setWebSecurityEnabled(bool);
    virtual void setJavaScriptCanOpenWindowsAutomatically(bool);
    virtual void setLoadsImagesAutomatically(bool);
    virtual void setImagesEnabled(bool);
    virtual void setPluginsEnabled(bool);
    virtual void setDOMPasteAllowed(bool);
    virtual void setDeveloperExtrasEnabled(bool);
    virtual void setNeedsSiteSpecificQuirks(bool);
    virtual void setShrinksStandaloneImagesToFit(bool);
    virtual void setUsesEncodingDetector(bool);
    virtual void setTextAreasAreResizable(bool);
    virtual void setJavaEnabled(bool);
    virtual void setAllowScriptsToCloseWindows(bool);
    virtual void setUserStyleSheetLocation(const WebURL&);
    virtual void setUsesPageCache(bool);
    virtual void setDownloadableBinaryFontsEnabled(bool);
    virtual void setXSSAuditorEnabled(bool);
    virtual void setLocalStorageEnabled(bool);
    virtual void setEditableLinkBehaviorNeverLive();
    virtual void setFontRenderingModeNormal();
    virtual void setShouldPaintCustomScrollbars(bool);
    virtual void setDatabasesEnabled(bool);
    virtual void setAllowUniversalAccessFromFileURLs(bool);
    virtual void setTextDirectionSubmenuInclusionBehaviorNeverIncluded();
    virtual void setOfflineWebApplicationCacheEnabled(bool);
    virtual void setExperimentalWebGLEnabled(bool);
    virtual void setGeolocationEnabled(bool);

private:
    WebCore::Settings* m_settings;
};

} // namespace WebKit

#endif
