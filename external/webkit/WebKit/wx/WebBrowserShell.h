
 
#ifndef WXWEBBROWSERSHELL_H
#define WXWEBBROWSERSHELL_H

#include "wx/wxprec.h"
#ifndef WX_PRECOMP
    #include "wx/wx.h"
#endif

#include "WebKitDefines.h"
#include "WebView.h"
#include <wx/srchctrl.h>

class WXDLLIMPEXP_WEBKIT wxWebBrowserShell : public wxFrame
{
public:
    // ctor(s)
#if SWIG
    %pythonAppend wxWebBrowserShell "self._setOORInfo(self)"
#endif
    wxWebBrowserShell(const wxString& title);

#ifndef SWIG
    ~wxWebBrowserShell();
#endif

    void ShowDebugMenu(bool show = true);
    wxWebView* webview;

protected:

    // event handlers (these functions should _not_ be virtual)
    void OnCut(wxCommandEvent& event);
    void OnCopy(wxCommandEvent& event);
    void OnPaste(wxCommandEvent& event);
    void OnQuit(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);
    void OnLoadFile(wxCommandEvent& event);
    void OnAddressBarEnter(wxCommandEvent& event);
    void OnSearchCtrlEnter(wxCommandEvent& event);
    void OnLoadEvent(wxWebViewLoadEvent& event);
    void OnBeforeLoad(wxWebViewBeforeLoadEvent& event);
    void OnBack(wxCommandEvent& event);
    void OnForward(wxCommandEvent& event);
    void OnStop(wxCommandEvent& event);
    void OnReload(wxCommandEvent& event);
    void OnBrowse(wxCommandEvent& event);
    void OnEdit(wxCommandEvent& event);
    
    void OnMakeTextLarger(wxCommandEvent& event);
    void OnMakeTextSmaller(wxCommandEvent& event);
    void OnGetSource(wxCommandEvent& event);
    
    // debug menu items
    void OnSetSource(wxCommandEvent& event);
    void OnRunScript(wxCommandEvent& myEvent);

private:
    wxTextCtrl* addressBar;
    wxSearchCtrl* searchCtrl;

    bool m_checkBeforeLoad;
    wxMenu* m_debugMenu;
    // any class wishing to process wxWindows events must use this macro
#ifndef SWIG
    DECLARE_EVENT_TABLE()
#endif
};

class WXDLLIMPEXP_WEBKIT wxPageSourceViewFrame : public wxFrame
{
public:
    wxPageSourceViewFrame(const wxString& source);
};

#endif // ifndef WXWEBBROWSERSHELL_H
