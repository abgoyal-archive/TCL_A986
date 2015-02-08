
 
#ifndef WXWEBFRAME_H
#define WXWEBFRAME_H

#include "wx/wxprec.h"
#ifndef WX_PRECOMP
    #include "wx/wx.h"
#endif

#include "WebKitDefines.h"

class WebFramePrivate;
class WebViewFrameData;
class wxWebView;

namespace WebCore {
    class ChromeClientWx;
    class FrameLoaderClientWx;
    class EditorClientWx;
    class Frame;
}

class WXDLLIMPEXP_WEBKIT wxWebViewDOMElementInfo
{
public:
    wxWebViewDOMElementInfo();

    ~wxWebViewDOMElementInfo() { }

    wxString GetTagName() const { return m_tagName; }
    void SetTagName(const wxString& name) { m_tagName = name; }

    bool IsSelected() const { return m_isSelected; }
    void SetSelected(bool sel) { m_isSelected = sel; }

    wxString GetText() const { return m_text; }
    void SetText(const wxString& text) { m_text = text; }

    wxString GetImageSrc() const { return m_imageSrc; }
    void SetImageSrc(const wxString& src) { m_imageSrc = src; }

    wxString GetLink() const { return m_link; }
    void SetLink(const wxString& link) { m_link = link; }

private:
    void* m_domElement;
    bool m_isSelected;
    wxString m_tagName;
    wxString m_text;
    wxString m_imageSrc;
    wxString m_link;
};

// based on enums in WebCore/dom/Document.h
enum wxWebKitParseMode { Compat, AlmostStrict, Strict, NoDocument };

class WXDLLIMPEXP_WEBKIT wxWebFrame
{
    // ChromeClientWx needs to get the Page* stored by the wxWebView
    // for the createWindow function. 
    friend class WebCore::ChromeClientWx;
    friend class WebCore::FrameLoaderClientWx;
    friend class WebCore::EditorClientWx;
    friend class wxWebView;

public:
    wxWebFrame(wxWebView* container, wxWebFrame* parent = NULL, WebViewFrameData* data = NULL);
    
    ~wxWebFrame();
    
    void LoadURL(const wxString& url);
    bool GoBack();
    bool GoForward();
    void Stop();
    void Reload();
    
    bool CanGoBack();
    bool CanGoForward();
    
    bool CanCut();
    bool CanCopy();
    bool CanPaste();
    
    void Cut();
    void Copy();
    void Paste();
    
    bool CanUndo();
    bool CanRedo();
    
    void Undo();
    void Redo();
    
    wxString GetPageSource();
    void SetPageSource(const wxString& source, const wxString& baseUrl = wxEmptyString);
    
    wxString GetInnerText();
    wxString GetAsMarkup();
    wxString GetExternalRepresentation();
    
    wxString RunScript(const wxString& javascript);
    
    bool FindString(const wxString& string, bool forward = true,
        bool caseSensitive = false, bool wrapSelection = true,
        bool startInSelection = true);
    
    bool CanIncreaseTextSize() const;
    void IncreaseTextSize();
    bool CanDecreaseTextSize() const;
    void DecreaseTextSize();
    void ResetTextSize();
    void MakeEditable(bool enable);
    bool IsEditable() const { return m_isEditable; }
    
    WebCore::Frame* GetFrame();

    wxWebViewDOMElementInfo HitTest(const wxPoint& post) const;
    
    bool ShouldClose() const;
    
    wxWebKitParseMode GetParseMode() const;
    
private:
    float m_textMagnifier;
    bool m_isEditable;
    bool m_isInitialized;
    bool m_beingDestroyed;
    WebFramePrivate* m_impl;
    
};

#endif // ifndef WXWEBFRAME_H
