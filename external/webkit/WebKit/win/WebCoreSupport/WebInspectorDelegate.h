

#ifndef WebInspectorDelegate_h
#define WebInspectorDelegate_h

struct IDataObject;
struct IPropertyBag;
struct IWebView;
struct IWebFrame;
struct IWebError;
struct IWebURLRequest;
struct IWebOpenPanelResultListener;

class WebInspectorDelegate : public IWebUIDelegate {
public:
    static WebInspectorDelegate* createInstance();

    // IUnknown
    virtual ULONG STDMETHODCALLTYPE AddRef();
    virtual ULONG STDMETHODCALLTYPE Release();
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID, void**) { return E_NOTIMPL; };

    // IWebUIDelegate
    virtual HRESULT STDMETHODCALLTYPE dragDestinationActionMaskForDraggingInfo( 
        /* [in] */ IWebView*,
        /* [in] */ IDataObject*,
        /* [retval][out] */ WebDragDestinationAction* action);

    // Not implemented
    virtual HRESULT STDMETHODCALLTYPE createWebViewWithRequest( 
        /* [in] */ IWebView*,
        /* [in] */ IWebURLRequest*,
        /* [retval][out] */ IWebView**) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE webViewShow( 
        /* [in] */ IWebView*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE webViewClose( 
        /* [in] */ IWebView*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE webViewFocus( 
        /* [in] */ IWebView*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE webViewUnfocus( 
        /* [in] */ IWebView*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE webViewFirstResponder( 
        /* [in] */ IWebView*,
        /* [retval][out] */ OLE_HANDLE*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE makeFirstResponder( 
        /* [in] */ IWebView*,
        /* [in] */ OLE_HANDLE) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE setStatusText( 
        /* [in] */ IWebView*,
        /* [in] */ BSTR) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE webViewStatusText( 
        /* [in] */ IWebView*,
        /* [retval][out] */ BSTR*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE webViewAreToolbarsVisible( 
        /* [in] */ IWebView*,
        /* [retval][out] */ BOOL*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE setToolbarsVisible( 
        /* [in] */ IWebView*,
        /* [in] */ BOOL) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE webViewIsStatusBarVisible( 
        /* [in] */ IWebView*,
        /* [retval][out] */ BOOL*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE setStatusBarVisible( 
        /* [in] */ IWebView*,
        /* [in] */ BOOL) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE webViewIsResizable( 
        /* [in] */ IWebView*,
        /* [retval][out] */ BOOL*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE setResizable( 
        /* [in] */ IWebView*,
        /* [in] */ BOOL) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE setFrame( 
        /* [in] */ IWebView*,
        /* [in] */ RECT*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE webViewFrame( 
        /* [in] */ IWebView*,
        /* [retval][out] */ RECT*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE setContentRect( 
        /* [in] */ IWebView*,
        /* [in] */ RECT*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE webViewContentRect( 
        /* [in] */ IWebView*,
        /* [retval][out] */ RECT*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE runJavaScriptAlertPanelWithMessage( 
        /* [in] */ IWebView*,
        /* [in] */ BSTR) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE runJavaScriptConfirmPanelWithMessage( 
        /* [in] */ IWebView*,
        /* [in] */ BSTR,
        /* [retval][out] */ BOOL*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE runJavaScriptTextInputPanelWithPrompt( 
        /* [in] */ IWebView*,
        /* [in] */ BSTR /*message*/,
        /* [in] */ BSTR /*defaultText*/,
        /* [retval][out] */ BSTR*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE runBeforeUnloadConfirmPanelWithMessage( 
        /* [in] */ IWebView*,
        /* [in] */ BSTR /*message*/,
        /* [in] */ IWebFrame*  /*initiatedByFrame*/,
        /* [retval][out] */ BOOL*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE runOpenPanelForFileButtonWithResultListener( 
        /* [in] */ IWebView*,
        /* [in] */ IWebOpenPanelResultListener*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE mouseDidMoveOverElement( 
        /* [in] */ IWebView*,
        /* [in] */ IPropertyBag*,
        /* [in] */ UINT /*modifierFlags*/) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE contextMenuItemsForElement( 
        /* [in] */ IWebView*,
        /* [in] */ IPropertyBag*,
        /* [in] */ OLE_HANDLE,
        /* [retval][out] */ OLE_HANDLE*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE validateUserInterfaceItem( 
        /* [in] */ IWebView*,
        /* [in] */ UINT,
        /* [in] */ BOOL,
        /* [retval][out] */ BOOL*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE shouldPerformAction( 
        /* [in] */ IWebView*,
        /* [in] */ UINT /*itemCommandID*/,
        /* [in] */ UINT /*sender*/) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE willPerformDragDestinationAction( 
        /* [in] */ IWebView*,
        /* [in] */ WebDragDestinationAction,
        /* [in] */ IDataObject*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE dragSourceActionMaskForPoint( 
        /* [in] */ IWebView*,
        /* [in] */ LPPOINT,
        /* [retval][out] */ WebDragSourceAction*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE willPerformDragSourceAction( 
        /* [in] */ IWebView*,
        /* [in] */ WebDragSourceAction,
        /* [in] */ LPPOINT,
        /* [in] */ IDataObject*,
        /* [retval][out] */ IDataObject**) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE contextMenuItemSelected( 
        /* [in] */ IWebView*,
        /* [in] */ void*  /*item*/,
        /* [in] */ IPropertyBag*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE hasCustomMenuImplementation( 
        /* [retval][out] */ BOOL*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE trackCustomPopupMenu( 
        /* [in] */ IWebView*,
        /* [in] */ OLE_HANDLE,
        /* [in] */ LPPOINT) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE measureCustomMenuItem( 
        /* [in] */ IWebView*,
        /* [in] */ void*  /*measureItem*/) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE drawCustomMenuItem( 
        /* [in] */ IWebView*,
        /* [in] */ void*  /*drawItem*/) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE addCustomMenuDrawingData( 
        /* [in] */ IWebView*,
        /* [in] */ OLE_HANDLE) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE cleanUpCustomMenuDrawingData( 
        /* [in] */ IWebView*,
        /* [in] */ OLE_HANDLE) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE canTakeFocus( 
        /* [in] */ IWebView*,
        /* [in] */ BOOL /*forward*/,
        /* [out] */ BOOL*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE takeFocus( 
        /* [in] */ IWebView*,
        /* [in] */ BOOL /*forward*/) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE registerUndoWithTarget( 
        /* [in] */ IWebUndoTarget*,
        /* [in] */ BSTR /*actionName*/,
        /* [in] */ IUnknown*  /*actionArg*/) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE removeAllActionsWithTarget( 
        /* [in] */ IWebUndoTarget*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE setActionTitle( 
        /* [in] */ BSTR) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE undo() { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE redo() { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE canUndo( 
        /* [retval][out] */ BOOL*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE canRedo( 
        /* [retval][out] */ BOOL*) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE printFrame( 
        /* [in] */ IWebView *webView,
        /* [in] */ IWebFrame *frame) { return E_NOTIMPL; }
    
    virtual HRESULT STDMETHODCALLTYPE ftpDirectoryTemplatePath( 
        /* [in] */ IWebView *webView,
        /* [retval][out] */ BSTR *path) { return E_NOTIMPL; }
    
    virtual HRESULT STDMETHODCALLTYPE webViewHeaderHeight( 
        /* [in] */ IWebView *webView,
        /* [retval][out] */ float *result) { return E_NOTIMPL; }
    
    virtual HRESULT STDMETHODCALLTYPE webViewFooterHeight( 
        /* [in] */ IWebView *webView,
        /* [retval][out] */ float *result) { return E_NOTIMPL; }
    
    virtual HRESULT STDMETHODCALLTYPE drawHeaderInRect( 
        /* [in] */ IWebView *webView,
        /* [in] */ RECT *rect,
        /* [in] */ OLE_HANDLE drawingContext) { return E_NOTIMPL; }
    
    virtual HRESULT STDMETHODCALLTYPE drawFooterInRect( 
        /* [in] */ IWebView *webView,
        /* [in] */ RECT *rect,
        /* [in] */ OLE_HANDLE drawingContext,
        /* [in] */ UINT pageIndex,
        /* [in] */ UINT pageCount) { return E_NOTIMPL; }
    
    virtual HRESULT STDMETHODCALLTYPE webViewPrintingMarginRect( 
        /* [in] */ IWebView *webView,
        /* [retval][out] */ RECT *rect) { return E_NOTIMPL; }
    
    virtual HRESULT STDMETHODCALLTYPE canRunModal( 
        /* [in] */ IWebView *webView,
        /* [retval][out] */ BOOL *canRunBoolean) { return E_NOTIMPL; }
    
    virtual HRESULT STDMETHODCALLTYPE createModalDialog( 
        /* [in] */ IWebView *sender,
        /* [in] */ IWebURLRequest *request,
        /* [retval][out] */ IWebView **newWebView) { return E_NOTIMPL; }
    
    virtual HRESULT STDMETHODCALLTYPE runModal( 
        /* [in] */ IWebView *webView) { return E_NOTIMPL; }
    
    virtual HRESULT STDMETHODCALLTYPE isMenuBarVisible( 
        /* [in] */ IWebView *webView,
        /* [retval][out] */ BOOL *visible) { return E_NOTIMPL; }
    
    virtual HRESULT STDMETHODCALLTYPE setMenuBarVisible( 
        /* [in] */ IWebView *webView,
        /* [in] */ BOOL visible) { return E_NOTIMPL; }
    
    virtual HRESULT STDMETHODCALLTYPE runDatabaseSizeLimitPrompt( 
        /* [in] */ IWebView *webView,
        /* [in] */ BSTR displayName,
        /* [in] */ IWebFrame *initiatedByFrame,
        /* [retval][out] */ BOOL *allowed) { return E_NOTIMPL; }
    
    virtual HRESULT STDMETHODCALLTYPE paintCustomScrollbar( 
        /* [in] */ IWebView *webView,
        /* [in] */ HDC hDC,
        /* [in] */ RECT rect,
        /* [in] */ WebScrollBarControlSize size,
        /* [in] */ WebScrollbarControlState state,
        /* [in] */ WebScrollbarControlPart pressedPart,
        /* [in] */ BOOL vertical,
        /* [in] */ float value,
        /* [in] */ float proportion,
        /* [in] */ WebScrollbarControlPartMask parts) { return E_NOTIMPL; }
    
    virtual HRESULT STDMETHODCALLTYPE paintCustomScrollCorner( 
        /* [in] */ IWebView *webView,
        /* [in] */ HDC hDC,
        /* [in] */ RECT rect) { return E_NOTIMPL; }

    virtual HRESULT STDMETHODCALLTYPE desktopNotificationsDelegate(
        /* [retval][out] */ IWebDesktopNotificationsDelegate **result) { return E_NOTIMPL; }

private:
    WebInspectorDelegate();

    ULONG m_refCount;
};

#endif // WebInspectorDelegate_h
