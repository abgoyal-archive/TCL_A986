

#ifndef BackForwardList_h
#define BackForwardList_h

#include <wtf/RefCounted.h>
#include <wtf/PassRefPtr.h>
#include <wtf/HashSet.h>
#include <wtf/Vector.h>

namespace WebCore {

class Document;
class HistoryItem;
class Page;
class SerializedScriptValue;
class String;

typedef Vector<RefPtr<HistoryItem> > HistoryItemVector;
typedef HashSet<RefPtr<HistoryItem> > HistoryItemHashSet;

#if PLATFORM(CHROMIUM)
// In the Chromium port, the back/forward list is managed externally.
// See BackForwardListChromium.cpp
class BackForwardListClient {
public:
    virtual ~BackForwardListClient() {}
    virtual void addItem(PassRefPtr<HistoryItem>) = 0;
    virtual void goToItem(HistoryItem*) = 0;
    virtual HistoryItem* currentItem() = 0;
    virtual HistoryItem* itemAtIndex(int) = 0;
    virtual int backListCount() = 0;
    virtual int forwardListCount() = 0;
    virtual void close() = 0;
};
#endif

class BackForwardList : public RefCounted<BackForwardList> {
public: 
    static PassRefPtr<BackForwardList> create(Page* page) { return adoptRef(new BackForwardList(page)); }
    ~BackForwardList();

#if PLATFORM(CHROMIUM)
    // Must be called before any other methods. 
    void setClient(BackForwardListClient* client) { m_client = client; }
#endif
    
    Page* page() { return m_page; }
    
    void addItem(PassRefPtr<HistoryItem>);
    void goBack();
    void goForward();
    void goToItem(HistoryItem*);
        
    HistoryItem* backItem();
    HistoryItem* currentItem();
    HistoryItem* forwardItem();
    HistoryItem* itemAtIndex(int);

    void backListWithLimit(int, HistoryItemVector&);
    void forwardListWithLimit(int, HistoryItemVector&);

    int capacity();
    void setCapacity(int);
    bool enabled();
    void setEnabled(bool);
    int backListCount();
    int forwardListCount();
    bool containsItem(HistoryItem*);

    void close();
    bool closed();
    
    void removeItem(HistoryItem*);
    HistoryItemVector& entries();
    
    void pushStateItem(PassRefPtr<HistoryItem>);

#if ENABLE(WML)
    void clearWMLPageHistory();
#endif

private:
    BackForwardList(Page*);
    
    Page* m_page;
#if PLATFORM(CHROMIUM) 
    BackForwardListClient* m_client;
#else
    HistoryItemVector m_entries;
    HistoryItemHashSet m_entryHash;
    unsigned m_current;
#endif
    unsigned m_capacity;
    bool m_closed;
    bool m_enabled;
};
    
} //namespace WebCore

#endif
