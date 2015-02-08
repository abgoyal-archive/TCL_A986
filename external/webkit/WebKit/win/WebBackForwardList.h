

#ifndef WebBackForwardList_H
#define WebBackForwardList_H

#include "WebKit.h"

#include "WebHistoryItem.h"

#include <WTF/PassRefPtr.h>
#include <WTF/RefPtr.h>

namespace WebCore {
    class BackForwardList;
}

class WebBackForwardList : public IWebBackForwardList, IWebBackForwardListPrivate
{
public:
    static WebBackForwardList* createInstance(PassRefPtr<WebCore::BackForwardList>);
protected:
    WebBackForwardList(PassRefPtr<WebCore::BackForwardList>);
    ~WebBackForwardList();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebBackForwardList
    virtual HRESULT STDMETHODCALLTYPE addItem( 
        /* [in] */ IWebHistoryItem *item);
    
    virtual HRESULT STDMETHODCALLTYPE goBack( void);
    
    virtual HRESULT STDMETHODCALLTYPE goForward( void);
    
    virtual HRESULT STDMETHODCALLTYPE goToItem( 
        /* [in] */ IWebHistoryItem *item);
    
    virtual HRESULT STDMETHODCALLTYPE backItem( 
        /* [retval][out] */ IWebHistoryItem **item);
    
    virtual HRESULT STDMETHODCALLTYPE currentItem( 
        /* [retval][out] */ IWebHistoryItem **item);
    
    virtual HRESULT STDMETHODCALLTYPE forwardItem( 
        /* [retval][out] */ IWebHistoryItem **item);
    
    virtual HRESULT STDMETHODCALLTYPE backListWithLimit( 
        /* [in] */ int limit,
        /* [out] */ int *listCount,
        /* [in] */ IWebHistoryItem **list);
    
    virtual HRESULT STDMETHODCALLTYPE forwardListWithLimit( 
        /* [in] */ int limit,
        /* [out] */ int *listCount,
        /* [in] */ IWebHistoryItem **list);
    
    virtual HRESULT STDMETHODCALLTYPE capacity( 
        /* [retval][out] */ int *result);
    
    virtual HRESULT STDMETHODCALLTYPE setCapacity( 
        /* [in] */ int size);
    
    virtual HRESULT STDMETHODCALLTYPE backListCount( 
        /* [retval][out] */ int *count);
    
    virtual HRESULT STDMETHODCALLTYPE forwardListCount( 
        /* [retval][out] */ int *sizecount);
    
    virtual HRESULT STDMETHODCALLTYPE containsItem( 
        /* [in] */ IWebHistoryItem *item,
        /* [retval][out] */ BOOL *result);
    
    virtual HRESULT STDMETHODCALLTYPE itemAtIndex( 
        /* [in] */ int index,
        /* [retval][out] */ IWebHistoryItem **item);
    
    // IWebBackForwardListPrivate
    virtual HRESULT STDMETHODCALLTYPE removeItem( 
        /* [in] */ IWebHistoryItem* item);

    // WebBackForwardList

protected:
    ULONG m_refCount;
    RefPtr<WebCore::BackForwardList> m_backForwardList;
};

#endif
