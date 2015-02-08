

#ifndef WebHistoryItem_H
#define WebHistoryItem_H

#include "WebKit.h"

#include <CoreFoundation/CoreFoundation.h>
#include <WebCore/PlatformString.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {
    class HistoryItem;
}

//-----------------------------------------------------------------------------

class WebHistoryItem : public IWebHistoryItem, IWebHistoryItemPrivate
{
public:
    static WebHistoryItem* createInstance();
    static WebHistoryItem* createInstance(PassRefPtr<WebCore::HistoryItem>);
protected:
    WebHistoryItem(PassRefPtr<WebCore::HistoryItem>);
    ~WebHistoryItem();

public:
    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebHistoryItem
    virtual HRESULT STDMETHODCALLTYPE initWithURLString( 
        /* [in] */ BSTR urlString,
        /* [in] */ BSTR title,
        /* [in] */ DATE lastVisited);
    
    virtual HRESULT STDMETHODCALLTYPE originalURLString( 
        /* [retval][out] */ BSTR *url);
    
    virtual HRESULT STDMETHODCALLTYPE URLString( 
        /* [retval][out] */ BSTR *url);
    
    virtual HRESULT STDMETHODCALLTYPE title( 
        /* [retval][out] */ BSTR *pageTitle);
    
    virtual HRESULT STDMETHODCALLTYPE lastVisitedTimeInterval( 
        /* [retval][out] */ DATE *lastVisited);
    
    virtual HRESULT STDMETHODCALLTYPE setAlternateTitle( 
        /* [in] */ BSTR title);
    
    virtual HRESULT STDMETHODCALLTYPE alternateTitle( 
        /* [retval][out] */ BSTR* title);
    
    virtual HRESULT STDMETHODCALLTYPE icon( 
        /* [out, retval] */ OLE_HANDLE *hBitmap);

    // IWebHistoryItemPrivate
    virtual HRESULT STDMETHODCALLTYPE initFromDictionaryRepresentation(void* dictionary);
    virtual HRESULT STDMETHODCALLTYPE dictionaryRepresentation(void** dictionary);
    virtual HRESULT STDMETHODCALLTYPE visitCount(int *count);
    virtual HRESULT STDMETHODCALLTYPE setVisitCount(int count);
    virtual HRESULT STDMETHODCALLTYPE hasURLString(BOOL* hasURL);
    virtual HRESULT STDMETHODCALLTYPE mergeAutoCompleteHints(IWebHistoryItem* otherItem);
    virtual HRESULT STDMETHODCALLTYPE setLastVisitedTimeInterval(DATE time);
    virtual HRESULT STDMETHODCALLTYPE setTitle(BSTR title);
    virtual HRESULT STDMETHODCALLTYPE RSSFeedReferrer(BSTR* url);
    virtual HRESULT STDMETHODCALLTYPE setRSSFeedReferrer(BSTR url);
    virtual HRESULT STDMETHODCALLTYPE hasPageCache(BOOL *hasCache);
    virtual HRESULT STDMETHODCALLTYPE setHasPageCache(BOOL hasCache);
    virtual HRESULT STDMETHODCALLTYPE target(BSTR* target);
    virtual HRESULT STDMETHODCALLTYPE isTargetItem(BOOL* result);
    virtual HRESULT STDMETHODCALLTYPE children(unsigned* childCount, SAFEARRAY** children);
    virtual HRESULT STDMETHODCALLTYPE lastVisitWasFailure(BOOL* wasFailure);
    virtual HRESULT STDMETHODCALLTYPE setLastVisitWasFailure(BOOL wasFailure);
    virtual HRESULT STDMETHODCALLTYPE lastVisitWasHTTPNonGet(BOOL* HTTPNonGet);
    virtual HRESULT STDMETHODCALLTYPE setLastVisitWasHTTPNonGet(BOOL HTTPNonGet);
    virtual HRESULT STDMETHODCALLTYPE redirectURLs(IEnumVARIANT**);
    virtual HRESULT STDMETHODCALLTYPE visitedWithTitle(BSTR title, BOOL increaseVisitCount);
    virtual HRESULT STDMETHODCALLTYPE getDailyVisitCounts(int* number, int** counts);
    virtual HRESULT STDMETHODCALLTYPE getWeeklyVisitCounts(int* number, int** counts);
    virtual HRESULT STDMETHODCALLTYPE recordInitialVisit();
    // WebHistoryItem
    WebCore::HistoryItem* historyItem() const;

protected:
    ULONG m_refCount;

    RefPtr<WebCore::HistoryItem> m_historyItem;
    WebCore::String m_alternateTitle;
};

#endif
