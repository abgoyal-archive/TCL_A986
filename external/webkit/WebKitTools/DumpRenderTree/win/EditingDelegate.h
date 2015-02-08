

#ifndef EditingDelegate_h
#define EditingDelegate_h

#include <WebKit/WebKit.h>

class __declspec(uuid("265DCD4B-79C3-44a2-84BC-511C3EDABD6F")) EditingDelegate : public IWebEditingDelegate {
public:
    EditingDelegate();

    void setAcceptsEditing(bool b) { m_acceptsEditing = b; }

    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebEditingDelegate
    virtual HRESULT STDMETHODCALLTYPE shouldBeginEditingInDOMRange( 
        /* [in] */ IWebView *webView,
        /* [in] */ IDOMRange *range,
        /* [retval][out] */ BOOL *result);
    
    virtual HRESULT STDMETHODCALLTYPE shouldEndEditingInDOMRange( 
        /* [in] */ IWebView *webView,
        /* [in] */ IDOMRange *range,
        /* [retval][out] */ BOOL *result);
    
    virtual HRESULT STDMETHODCALLTYPE shouldInsertNode( 
        /* [in] */ IWebView *webView,
        /* [in] */ IDOMNode *node,
        /* [in] */ IDOMRange *range,
        /* [in] */ WebViewInsertAction action);
    
    virtual HRESULT STDMETHODCALLTYPE shouldInsertText( 
        /* [in] */ IWebView *webView,
        /* [in] */ BSTR text,
        /* [in] */ IDOMRange *range,
        /* [in] */ WebViewInsertAction action,
        /* [retval][out] */ BOOL *result);
    
    virtual HRESULT STDMETHODCALLTYPE shouldDeleteDOMRange( 
        /* [in] */ IWebView *webView,
        /* [in] */ IDOMRange *range,
        /* [retval][out] */ BOOL *result);
    
    virtual HRESULT STDMETHODCALLTYPE shouldChangeSelectedDOMRange( 
        /* [in] */ IWebView *webView,
        /* [in] */ IDOMRange *currentRange,
        /* [in] */ IDOMRange *proposedRange,
        /* [in] */ WebSelectionAffinity selectionAffinity,
        /* [in] */ BOOL stillSelecting,
        /* [retval][out] */ BOOL *result);
    
    virtual HRESULT STDMETHODCALLTYPE shouldApplyStyle( 
        /* [in] */ IWebView *webView,
        /* [in] */ IDOMCSSStyleDeclaration *style,
        /* [in] */ IDOMRange *range,
        /* [retval][out] */ BOOL *result);
    
    virtual HRESULT STDMETHODCALLTYPE shouldChangeTypingStyle( 
        /* [in] */ IWebView *webView,
        /* [in] */ IDOMCSSStyleDeclaration *currentStyle,
        /* [in] */ IDOMCSSStyleDeclaration *proposedStyle,
        /* [retval][out] */ BOOL *result);
    
    virtual HRESULT STDMETHODCALLTYPE doPlatformCommand( 
        /* [in] */ IWebView *webView,
        /* [in] */ BSTR command,
        /* [retval][out] */ BOOL *result);
    
    virtual HRESULT STDMETHODCALLTYPE webViewDidBeginEditing( 
        /* [in] */ IWebNotification *notification);
    
    virtual HRESULT STDMETHODCALLTYPE webViewDidChange( 
        /* [in] */ IWebNotification *notification);
    
    virtual HRESULT STDMETHODCALLTYPE webViewDidEndEditing( 
        /* [in] */ IWebNotification *notification);
    
    virtual HRESULT STDMETHODCALLTYPE webViewDidChangeTypingStyle( 
        /* [in] */ IWebNotification *notification);
    
    virtual HRESULT STDMETHODCALLTYPE webViewDidChangeSelection( 
        /* [in] */ IWebNotification *notification);
    
    virtual HRESULT STDMETHODCALLTYPE undoManagerForWebView( 
        /* [in] */ IWebView *webView,
        /* [retval][out] */ IWebUndoManager **undoManager) { return E_NOTIMPL; }

        virtual HRESULT STDMETHODCALLTYPE ignoreWordInSpellDocument( 
            /* [in] */ IWebView *view,
            /* [in] */ BSTR word) { return E_NOTIMPL; }
        
        virtual HRESULT STDMETHODCALLTYPE learnWord( 
            /* [in] */ BSTR word) { return E_NOTIMPL; }
        
        virtual HRESULT STDMETHODCALLTYPE checkSpellingOfString( 
            /* [in] */ IWebView *view,
            /* [in] */ LPCTSTR text,
            /* [in] */ int length,
            /* [out] */ int *misspellingLocation,
            /* [out] */ int *misspellingLength) { return E_NOTIMPL; }
        
        virtual HRESULT STDMETHODCALLTYPE checkGrammarOfString( 
            /* [in] */ IWebView *view,
            /* [in] */ LPCTSTR text,
            /* [in] */ int length,
            /* [out] */ IEnumWebGrammarDetails **grammarDetails,
            /* [out] */ int *badGrammarLocation,
            /* [out] */ int *badGrammarLength) { return E_NOTIMPL; }
        
        virtual HRESULT STDMETHODCALLTYPE updateSpellingUIWithGrammarString( 
            /* [in] */ BSTR string,
            /* [in] */ int location,
            /* [in] */ int length,
            /* [in] */ BSTR userDescription,
            /* [in] */ BSTR *guesses,
            /* [in] */ int guessesCount) { return E_NOTIMPL; }
        
        virtual HRESULT STDMETHODCALLTYPE updateSpellingUIWithMisspelledWord( 
            /* [in] */ BSTR word) { return E_NOTIMPL; }
        
        virtual HRESULT STDMETHODCALLTYPE showSpellingUI( 
            /* [in] */ BOOL show) { return E_NOTIMPL; }
        
        virtual HRESULT STDMETHODCALLTYPE spellingUIIsShowing( 
            /* [retval][out] */ BOOL *result) { return E_NOTIMPL; }
        
        virtual HRESULT STDMETHODCALLTYPE guessesForWord( 
            /* [in] */ BSTR word,
            /* [retval][out] */ IEnumSpellingGuesses **guesses) { return E_NOTIMPL; }
        
        virtual HRESULT STDMETHODCALLTYPE closeSpellDocument( 
            /* [in] */ IWebView *view) { return E_NOTIMPL; }
        
        virtual HRESULT STDMETHODCALLTYPE sharedSpellCheckerExists( 
            /* [retval][out] */ BOOL *exists) { return E_NOTIMPL; }
        
        virtual HRESULT STDMETHODCALLTYPE preflightChosenSpellServer( void) { return E_NOTIMPL; }
        
        virtual HRESULT STDMETHODCALLTYPE updateGrammar( void) { return E_NOTIMPL; }

private:
    bool m_acceptsEditing;
    ULONG m_refCount;
};

#endif // !defined(EditingDelegate_h)
