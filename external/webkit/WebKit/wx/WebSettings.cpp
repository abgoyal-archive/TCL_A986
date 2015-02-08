

#include "config.h"
#include "WebSettings.h"

#include "PlatformString.h"
#include "Settings.h"

void wxWebSettings::SetDefaultFixedFontSize(int size)
{
    if (m_settings)
        m_settings->setDefaultFixedFontSize(size);
}
    
int wxWebSettings::GetDefaultFixedFontSize() const
{
    if (m_settings)
        return m_settings->defaultFixedFontSize();
        
    return 0;
}
    
void wxWebSettings::SetDefaultFontSize(int size)
{
    if (m_settings)
        m_settings->setDefaultFontSize(size);
}

int wxWebSettings::GetDefaultFontSize() const
{
    if (m_settings)
        return m_settings->defaultFontSize();
        
    return 0;
}
    
void wxWebSettings::SetMinimumFontSize(int size)
{
    if (m_settings)
        m_settings->setMinimumFontSize(size);
}

int wxWebSettings::GetMinimumFontSize() const
{
    if (m_settings)
        return m_settings->minimumFontSize();
        
    return 0;
}
    
void wxWebSettings::SetLoadsImagesAutomatically(bool loadAutomatically)
{
    if (m_settings)
        m_settings->setLoadsImagesAutomatically(loadAutomatically);
}
    
bool wxWebSettings::LoadsImagesAutomatically() const
{
    if (m_settings)
        return m_settings->loadsImagesAutomatically();
        
    return false;
}
    
void wxWebSettings::SetJavaScriptEnabled(bool enabled)
{
    if (m_settings)
        m_settings->setJavaScriptEnabled(enabled);
}

bool wxWebSettings::IsJavaScriptEnabled() const
{
    if (m_settings)
        return m_settings->isJavaScriptEnabled();
        
    return false;
}
    
void wxWebSettings::SetDatabasesEnabled(bool enabled)
{
    if (m_settings)
        m_settings->setDatabasesEnabled(enabled);
}
    
bool wxWebSettings::AreDatabasesEnabled() const
{
    if (m_settings)
        return m_settings->databasesEnabled();
        
    return false;
}
    
void wxWebSettings::SetLocalStoragePath(const wxString& path)
{
    if (m_settings)
        m_settings->setLocalStorageDatabasePath(path);
}

wxString wxWebSettings::GetLocalStoragePath() const
{
    if (m_settings)
        return m_settings->localStorageDatabasePath();
        
    return wxEmptyString;
}

void wxWebSettings::SetEditableLinkBehavior(wxEditableLinkBehavior behavior)
{
    WebCore::EditableLinkBehavior webCoreBehavior;
    if (m_settings) {
        switch (behavior) {
        case wxEditableLinkAlwaysLive:
            webCoreBehavior = WebCore::EditableLinkAlwaysLive;
            break;
        case wxEditableLinkOnlyLiveWithShiftKey:
            webCoreBehavior = WebCore::EditableLinkOnlyLiveWithShiftKey;
            break;
        case wxEditableLinkLiveWhenNotFocused:
            webCoreBehavior = WebCore::EditableLinkLiveWhenNotFocused;
            break;
        case wxEditableLinkNeverLive:
            webCoreBehavior = WebCore::EditableLinkNeverLive;
            break;
        default:
            webCoreBehavior = WebCore::EditableLinkDefaultBehavior;
        }
        m_settings->setEditableLinkBehavior(webCoreBehavior);
    }
}

wxEditableLinkBehavior wxWebSettings::GetEditableLinkBehavior() const
{
    wxEditableLinkBehavior behavior = wxEditableLinkDefaultBehavior;
    if (m_settings) {
        WebCore::EditableLinkBehavior webCoreBehavior = m_settings->editableLinkBehavior();
        switch (webCoreBehavior) {
        case WebCore::EditableLinkAlwaysLive:
            behavior = wxEditableLinkAlwaysLive;
            break;
        case WebCore::EditableLinkOnlyLiveWithShiftKey:
            behavior = wxEditableLinkOnlyLiveWithShiftKey;
            break;
        case WebCore::EditableLinkLiveWhenNotFocused:
            behavior = wxEditableLinkLiveWhenNotFocused;
            break;
        case WebCore::EditableLinkNeverLive:
            behavior = wxEditableLinkNeverLive;
            break;
        default:
            behavior = wxEditableLinkDefaultBehavior;
        }
    }
    return behavior;
}

void wxWebSettings::SetPluginsEnabled(bool enabled)
{
    if (m_settings)
        m_settings->setPluginsEnabled(enabled);
}  

bool wxWebSettings::ArePluginsEnabled() const
{
    if (m_settings)
        return m_settings->arePluginsEnabled();
        
    return false;
}
