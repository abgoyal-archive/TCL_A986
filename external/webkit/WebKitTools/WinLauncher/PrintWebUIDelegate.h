

#ifndef PrintWebUIDelegate_h
#define PrintWebUIDelegate_h

#include <WebKit/WebKit.h>

class PrintWebUIDelegate : public IWebUIDelegate {
public:
    PrintWebUIDelegate() : m_refCount(1) {}

    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    virtual HRESULT STDMETHODCALLTYPE createWebViewWithRequest(IWebView*, IWebURLRequest*, IWebView**) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE webViewShow(IWebView*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE webViewClose(IWebView*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE webViewFocus(IWebView*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE webViewUnfocus(IWebView*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE webViewFirstResponder(IWebView*, OLE_HANDLE*)  { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE makeFirstResponder(IWebView*, OLE_HANDLE) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE setStatusText(IWebView*, BSTR) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE webViewStatusText(IWebView*, BSTR*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE webViewAreToolbarsVisible(IWebView*, BOOL*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE setToolbarsVisible(IWebView*, BOOL) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE webViewIsStatusBarVisible(IWebView*, BOOL*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE setStatusBarVisible(IWebView*, BOOL) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE webViewIsResizable(IWebView*, BOOL*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE setResizable(IWebView*, BOOL) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE setFrame(IWebView*, RECT*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE webViewFrame(IWebView*, RECT*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE setContentRect(IWebView*, RECT*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE webViewContentRect(IWebView*, RECT*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE runJavaScriptAlertPanelWithMessage(IWebView*, BSTR) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE runJavaScriptConfirmPanelWithMessage(IWebView*, BSTR, BOOL*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE runJavaScriptTextInputPanelWithPrompt(IWebView*, BSTR, BSTR, BSTR*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE runBeforeUnloadConfirmPanelWithMessage(IWebView*, BSTR, IWebFrame*, BOOL*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE runOpenPanelForFileButtonWithResultListener(IWebView*, IWebOpenPanelResultListener*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE mouseDidMoveOverElement(IWebView*, IPropertyBag*, UINT) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE contextMenuItemsForElement(IWebView*, IPropertyBag*, OLE_HANDLE, OLE_HANDLE*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE validateUserInterfaceItem(IWebView*, UINT, BOOL, BOOL*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE shouldPerformAction(IWebView*, UINT, UINT) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE dragDestinationActionMaskForDraggingInfo(IWebView*, IDataObject*, WebDragDestinationAction*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE willPerformDragDestinationAction(IWebView*, WebDragDestinationAction, IDataObject*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE dragSourceActionMaskForPoint(IWebView*, LPPOINT, WebDragSourceAction*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE willPerformDragSourceAction(IWebView*, WebDragSourceAction, LPPOINT, IDataObject*, IDataObject**) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE contextMenuItemSelected(IWebView*, void*, IPropertyBag*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE hasCustomMenuImplementation(BOOL*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE trackCustomPopupMenu(IWebView*, OLE_HANDLE, LPPOINT) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE measureCustomMenuItem(IWebView*, void*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE drawCustomMenuItem(IWebView*, void*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE addCustomMenuDrawingData(IWebView*, OLE_HANDLE) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE cleanUpCustomMenuDrawingData(IWebView*, OLE_HANDLE) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE canTakeFocus(IWebView*, BOOL, BOOL*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE takeFocus(IWebView*, BOOL) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE registerUndoWithTarget(IWebUndoTarget*, BSTR, IUnknown*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE removeAllActionsWithTarget(IWebUndoTarget*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE setActionTitle(BSTR) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE undo() { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE redo() { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE canUndo(BOOL*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE canRedo(BOOL*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE printFrame(IWebView*, IWebFrame *) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE ftpDirectoryTemplatePath(IWebView*, BSTR*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE webViewHeaderHeight(IWebView*, float*);
    virtual HRESULT STDMETHODCALLTYPE webViewFooterHeight(IWebView*, float*);
    virtual HRESULT STDMETHODCALLTYPE drawHeaderInRect(IWebView*, RECT*, OLE_HANDLE);
    virtual HRESULT STDMETHODCALLTYPE drawFooterInRect(IWebView*, RECT*, OLE_HANDLE, UINT, UINT);
    virtual HRESULT STDMETHODCALLTYPE webViewPrintingMarginRect(IWebView*, RECT*);
    virtual HRESULT STDMETHODCALLTYPE canRunModal(IWebView*, BOOL*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE createModalDialog(IWebView*, IWebURLRequest*, IWebView**) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE runModal(IWebView*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE isMenuBarVisible(IWebView*, BOOL*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE setMenuBarVisible(IWebView*, BOOL) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE runDatabaseSizeLimitPrompt(IWebView*, BSTR, IWebFrame*, BOOL*) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE paintCustomScrollbar(IWebView*, HDC, RECT, WebScrollBarControlSize, WebScrollbarControlState, WebScrollbarControlPart, BOOL, float, float, WebScrollbarControlPartMask) { return E_NOTIMPL; }
    virtual HRESULT STDMETHODCALLTYPE paintCustomScrollCorner(IWebView*, HDC, RECT) { return E_NOTIMPL; }

private:
    int m_refCount;
};

#endif
