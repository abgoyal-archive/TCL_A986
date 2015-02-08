
 
#ifndef WebSettings_h
#define WebSettings_h

#include "wx/wxprec.h"
#ifndef WX_PRECOMP
    #include "wx/wx.h"
#endif

#include "WebKitDefines.h"

namespace WebCore {
class Settings;
}

enum wxEditableLinkBehavior {
    wxEditableLinkDefaultBehavior,
    wxEditableLinkAlwaysLive,
    wxEditableLinkOnlyLiveWithShiftKey,
    wxEditableLinkLiveWhenNotFocused,
    wxEditableLinkNeverLive
};


class WXDLLIMPEXP_WEBKIT wxWebSettings: public wxObject {
public:
    wxWebSettings(WebCore::Settings* settings) :
        wxObject(),
        m_settings(settings)
    {}
    
    wxWebSettings() : wxObject() {}

    /**
        Sets the default font size for fixed fonts.
    */
    void SetDefaultFixedFontSize(int size);
    
    /**
        Returns the default font size for fixed fonts.
    */
    int GetDefaultFixedFontSize() const;
    
    /**
        Sets the default font size for fonts.
    */
    void SetDefaultFontSize(int size);

    /**
        Returns the default font size for fonts.
    */
    int GetDefaultFontSize() const;
    
    /**
        Sets the minimum acceptable font size.
    */
    void SetMinimumFontSize(int size);

    /**
        Returns the minimum acceptable font size.
    */
    int GetMinimumFontSize() const;
    
    /**
        Sets whether or not images are loaded automatically. (e.g. in email 
        programs you may wish to not load images until you confirm it is not SPAM)
    */
    void SetLoadsImagesAutomatically(bool loadAutomatically);
    
    /**
        Returns whether or not images are loaded automatically.
    */
    bool LoadsImagesAutomatically() const;
    
    /**
        Sets whether or not the WebView runs JavaScript code.
    */
    void SetJavaScriptEnabled(bool enabled);

    /**
        Returns whether or not the WebView runs JavaScript code.
    */
    bool IsJavaScriptEnabled() const;
    
    /**
        Sets whether or not web pages can create databases.
    */
    void SetDatabasesEnabled(bool enabled);
    
    /**
        Returns whether or not the WebView runs JavaScript code.
    */    
    bool AreDatabasesEnabled() const;
    
    /**
        Sets the path where local data will be stored.
    */
    void SetLocalStoragePath(const wxString& path);

    /**
        Returns the path where local data will be stored.
    */
    wxString GetLocalStoragePath() const;
    
    /**
        Sets how links are handled when the wxWebView is in editing mode. 
    */    
    void SetEditableLinkBehavior(wxEditableLinkBehavior behavior);
    
    /**
        Returns how links are handled when the wxWebView is in editing mode. 
    */   
    wxEditableLinkBehavior GetEditableLinkBehavior() const;
    
    /**
        Sets whether or not web pages can load plugins.
    */
    void SetPluginsEnabled(bool enabled);
    
    /**
        Returns whether or not web pages can load plugins.
    */    
    bool ArePluginsEnabled() const;
    
private:
    WebCore::Settings* m_settings;

};

#endif // WebSettings_h
