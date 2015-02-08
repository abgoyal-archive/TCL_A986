

#ifndef PageCache_h
#define PageCache_h

#include "HistoryItem.h"
#include "Timer.h"
#include <wtf/Forward.h>
#include <wtf/HashSet.h>
#include <wtf/Noncopyable.h>

namespace WebCore {

    class CachedPage;
    class HistoryItem;
    
    class PageCache : public Noncopyable {
    public:
        friend PageCache* pageCache();

        void setCapacity(int); // number of pages to cache
        int capacity() { return m_capacity; }
        
        void add(PassRefPtr<HistoryItem>, PassRefPtr<CachedPage>); // Prunes if capacity() is exceeded.
        void remove(HistoryItem*);
        CachedPage* get(HistoryItem* item) { return item ? item->m_cachedPage.get() : 0; }

        void releaseAutoreleasedPagesNow();
        
        int pageCount() const { return m_size; }
        int frameCount() const;
        int autoreleasedPageCount() const;

    private:
        typedef HashSet<RefPtr<CachedPage> > CachedPageSet;

        PageCache(); // Use pageCache() instead.
        ~PageCache(); // Not implemented to make sure nobody accidentally calls delete -- WebCore does not delete singletons.

        void addToLRUList(HistoryItem*); // Adds to the head of the list.
        void removeFromLRUList(HistoryItem*);

        void prune();

        void autorelease(PassRefPtr<CachedPage>);
        void releaseAutoreleasedPagesNowOrReschedule(Timer<PageCache>*);

        int m_capacity;
        int m_size;

        // LRU List
        HistoryItem* m_head;
        HistoryItem* m_tail;
        
        Timer<PageCache> m_autoreleaseTimer;
        CachedPageSet m_autoreleaseSet;
     };

    // Function to obtain the global page cache.
    PageCache* pageCache();

} // namespace WebCore

#endif // PageCache_h
