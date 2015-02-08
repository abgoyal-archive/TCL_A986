

#include "stdafx.h"
#include "PrintWebUIDelegate.h"

#include <commctrl.h>
#include <commdlg.h>
#include <objbase.h>
#include <shlwapi.h>
#include <wininet.h>

#include <WebKit/WebKitCOMAPI.h>

static const int MARGIN = 20;

HRESULT PrintWebUIDelegate::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualIID(riid, IID_IUnknown))
        *ppvObject = static_cast<IWebUIDelegate*>(this);
    else if (IsEqualIID(riid, IID_IWebUIDelegate))
        *ppvObject = static_cast<IWebUIDelegate*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG PrintWebUIDelegate::AddRef(void)
{
    return ++m_refCount;
}

ULONG PrintWebUIDelegate::Release(void)
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete this;

    return newRef;
}

HRESULT PrintWebUIDelegate::webViewPrintingMarginRect(IWebView* view, RECT* rect)
{
    if (!view || !rect)
        return E_POINTER;

    IWebFrame* mainFrame = 0;
    if (FAILED(view->mainFrame(&mainFrame)))
        return E_FAIL;

    IWebFramePrivate* privateFrame = 0;
    if (FAILED(mainFrame->QueryInterface(&privateFrame))) {
        mainFrame->Release();
        return E_FAIL;
    }

    privateFrame->frameBounds(rect);

    rect->left += MARGIN;
    rect->top += MARGIN;
    HDC dc = ::GetDC(0);
    rect->right = (::GetDeviceCaps(dc, LOGPIXELSX) * 6.5) - MARGIN;
    rect->bottom = (::GetDeviceCaps(dc, LOGPIXELSY) * 11) - MARGIN;
    ::ReleaseDC(0, dc);

    privateFrame->Release();
    mainFrame->Release();

    return S_OK;
}

HRESULT PrintWebUIDelegate::webViewHeaderHeight(IWebView* webView, float* height)
{
   if (!webView || !height)
      return E_POINTER;
   
   HDC dc = ::GetDC(0);

   TEXTMETRIC textMetric;
   ::GetTextMetrics(dc, &textMetric);
   ::ReleaseDC(0, dc);

   *height = 1.1 * textMetric.tmHeight;

   return S_OK;
}

HRESULT PrintWebUIDelegate::webViewFooterHeight(IWebView* webView, float* height)
{
   if (!webView || !height)
      return E_POINTER;

   HDC dc = ::GetDC(0);

   TEXTMETRIC textMetric;
   ::GetTextMetrics(dc, &textMetric);
   ::ReleaseDC(0, dc);

   *height = 1.1 * textMetric.tmHeight;

   return S_OK;
}

HRESULT PrintWebUIDelegate::drawHeaderInRect( 
            /* [in] */ IWebView *webView,
            /* [in] */ RECT *rect,
            /* [in] */ OLE_HANDLE drawingContext)
{
   if (!webView || !rect)
      return E_POINTER;

   // Turn off header for now.
   HDC dc = reinterpret_cast<HDC>(drawingContext);

   HFONT hFont = (HFONT)::GetStockObject(ANSI_VAR_FONT);
   HFONT hOldFont = (HFONT)::SelectObject(dc, hFont);

   LPCTSTR header(_T("[Sample Header]"));
   int length = _tcslen(header);

   int rc = ::DrawText(dc, header, length, rect, DT_LEFT | DT_NOCLIP | DT_VCENTER  | DT_SINGLELINE);
   ::SelectObject(dc, hOldFont);

   if (!rc)
      return E_FAIL;

   ::MoveToEx(dc, rect->left, rect->bottom, 0);
   HPEN hPen = (HPEN)::GetStockObject(BLACK_PEN);
   HPEN hOldPen = (HPEN)::SelectObject(dc, hPen);
   ::LineTo(dc, rect->right, rect->bottom);
   ::SelectObject(dc, hOldPen);

   return S_OK;
}

HRESULT PrintWebUIDelegate::drawFooterInRect( 
            /* [in] */ IWebView *webView,
            /* [in] */ RECT *rect,
            /* [in] */ OLE_HANDLE drawingContext,
            /* [in] */ UINT pageIndex,
            /* [in] */ UINT pageCount)
{
   if (!webView || !rect)
      return E_POINTER;

   HDC dc = reinterpret_cast<HDC>(drawingContext);

   HFONT hFont = (HFONT)::GetStockObject(ANSI_VAR_FONT);
   HFONT hOldFont = (HFONT)::SelectObject(dc, hFont);

   LPCTSTR footer(_T("[Sample Footer]"));
   int length = _tcslen(footer);

   // Add a line, 1/10th inch above the footer text from left margin to right margin.
   ::MoveToEx(dc, rect->left, rect->top, 0);
   HPEN hPen = (HPEN)::GetStockObject(BLACK_PEN);
   HPEN hOldPen = (HPEN)::SelectObject(dc, hPen);
   ::LineTo(dc, rect->right, rect->top);
   ::SelectObject(dc, hOldPen);

   int rc = ::DrawText(dc, footer, length, rect, DT_LEFT | DT_NOCLIP | DT_VCENTER  | DT_SINGLELINE);
   ::SelectObject(dc, hOldFont);

   if (!rc)
      return E_FAIL;

   return S_OK;
}
