
 
#ifndef PageURLRecord_h
#define PageURLRecord_h

#include "PlatformString.h"

#include <wtf/Noncopyable.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class IconRecord;

class PageURLSnapshot {
public:
    PageURLSnapshot() { }
    
    PageURLSnapshot(const String& page, const String& icon)
        : pageURL(page)
        , iconURL(icon)
    { }
    
    String pageURL;
    String iconURL;
};

class PageURLRecord : public Noncopyable {
public:
    PageURLRecord(const String& pageURL);
    ~PageURLRecord();

    inline String url() const { return m_pageURL; }
    
    void setIconRecord(PassRefPtr<IconRecord>);
    IconRecord* iconRecord() { return m_iconRecord.get(); }

    PageURLSnapshot snapshot(bool forDeletion = false) const;

    // Returns false if the page wasn't retained beforehand, true if the retain count was already 1 or higher
    inline bool retain() { return m_retainCount++; }

    // Returns true if the page is still retained after the call.  False if the retain count just dropped to 0
    inline bool release()
    {
        ASSERT(m_retainCount > 0);
        return --m_retainCount;
    }

    inline int retainCount() const { return m_retainCount; }
private:
    String m_pageURL;
    RefPtr<IconRecord> m_iconRecord;
    int m_retainCount;
};

}

#endif // PageURLRecord_h
