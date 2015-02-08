

#ifndef WebScrollBar_h
#define WebScrollBar_h

#include "WebKit.h"

#include <wtf/RefPtr.h>
#include <wtf/OwnPtr.h>

#pragma warning(push, 0)
#include <WebCore/COMPtr.h>
#include <WebCore/Scrollbar.h>
#include <WebCore/ScrollbarClient.h>
#pragma warning(pop)

namespace WebCore {
class Scrollbar;
}

using namespace WebCore;

class WebScrollBar : public IWebScrollBarPrivate, ScrollbarClient
{
public:
    static WebScrollBar* createInstance();
protected:
    WebScrollBar();
    ~WebScrollBar();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebScrollBarPrivate
    virtual HRESULT STDMETHODCALLTYPE init( 
        /* [in] */ IWebScrollBarDelegatePrivate* delegate,
        /* [in] */ OLE_HANDLE containingWindow,
        /* [in] */ WebScrollBarOrientation orientation,
        /* [in] */ WebScrollBarControlSize controlSize);
    
    virtual HRESULT STDMETHODCALLTYPE setEnabled( 
        /* [in] */ BOOL enabled);
    
    virtual HRESULT STDMETHODCALLTYPE setSteps( 
        /* [in] */ int lineStep,
        /* [in] */ int pageStep);
    
    virtual HRESULT STDMETHODCALLTYPE setProportion( 
        /* [in] */ int visibleSize,
        /* [in] */ int totalSize);
    
    virtual HRESULT STDMETHODCALLTYPE setRect( 
        /* [in] */ RECT bounds);
    
    virtual HRESULT STDMETHODCALLTYPE setValue( 
        /* [in] */ int value);
    
    virtual HRESULT STDMETHODCALLTYPE value( 
        /* [retval][out] */ int* value);
   
    virtual HRESULT STDMETHODCALLTYPE paint( 
        /* [in] */ HDC dc,
        /* [in] */ RECT damageRect);
    
    virtual HRESULT STDMETHODCALLTYPE frameRect( 
        /* [retval][out] */ RECT* bounds);
    
    virtual HRESULT STDMETHODCALLTYPE width( 
        /* [retval][out] */ int* w);
    
    virtual HRESULT STDMETHODCALLTYPE height( 
        /* [retval][out] */ int* h);
    
    virtual HRESULT STDMETHODCALLTYPE requestedWidth( 
        /* [retval][out] */ int* w);
    
    virtual HRESULT STDMETHODCALLTYPE requestedHeight( 
        /* [retval][out] */ int* h);

    virtual HRESULT STDMETHODCALLTYPE handleMouseEvent( 
        /* [in] */ OLE_HANDLE window,
        /* [in] */ UINT msg,
        /* [in] */ WPARAM wParam,
        /* [in] */ LPARAM lParam);
    
    virtual HRESULT STDMETHODCALLTYPE scroll( 
        /* [in] */ WebScrollDirection direction,
        /* [in] */ WebScrollGranularity granularity,
        /* [in] */ float multiplier);

protected:
    // ScrollbarClient
    virtual void valueChanged(Scrollbar*);
    virtual void invalidateScrollbarRect(Scrollbar*, const IntRect&);

    // FIXME: We should provide a way to set this value.
    virtual bool isActive() const { return true; }

    virtual bool scrollbarCornerPresent() const { return false; }

    ULONG m_refCount;
    HWND m_containingWindow;
    RefPtr<WebCore::Scrollbar> m_scrollBar;
    COMPtr<IWebScrollBarDelegatePrivate> m_delegate;
};

#endif
