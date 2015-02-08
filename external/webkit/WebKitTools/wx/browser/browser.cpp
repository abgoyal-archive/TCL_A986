
 
// webkit includes
#include "WebBrowserShell.h"
#include "WebSettings.h"
#include "WebView.h"

#include "wx/wxprec.h"
#ifndef WX_PRECOMP
    #include "wx/wx.h"
#endif

class MyApp : public wxApp
{
public:

    virtual bool OnInit();
};


IMPLEMENT_APP(MyApp)

bool MyApp::OnInit()
{
    wxInitAllImageHandlers();
        
    // create the main application window
    // see WebKit/wx/WebFrame.cpp for how to write a shell around wxWebView.
    wxWebBrowserShell *frame = new wxWebBrowserShell(_T("wxWebKit Test App"));

#ifndef NDEBUG
    frame->ShowDebugMenu(true);
#endif

    wxWebSettings settings = frame->webview->GetWebSettings();
#if __WXMSW__ || __WXMAC__
    settings.SetPluginsEnabled(true);
#endif
    settings.SetDatabasesEnabled(true);
    settings.SetEditableLinkBehavior(wxEditableLinkOnlyLiveWithShiftKey);
    frame->CentreOnScreen();
    frame->Show(true);

    return true;
}
