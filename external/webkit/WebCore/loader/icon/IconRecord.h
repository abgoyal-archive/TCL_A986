
 
#ifndef IconRecord_h
#define IconRecord_h

#include "PageURLRecord.h"
#include <wtf/RefCounted.h>
#include "SharedBuffer.h"

#include <wtf/HashSet.h>
#include <wtf/OwnPtr.h>
#include "PlatformString.h"
#include "StringHash.h"

namespace WebCore { 

class IconDataSnapshot;
class Image;
class IntSize;
class SQLDatabase;

enum ImageDataStatus {
    ImageDataStatusPresent, ImageDataStatusMissing, ImageDataStatusUnknown
};

class IconSnapshot {
public:
    IconSnapshot() : timestamp(0) { }
    
    IconSnapshot(const String& url, int stamp, SharedBuffer* theData)
        : iconURL(url)
        , timestamp(stamp)
        , data(theData)
    { }
        
    String iconURL;
    int timestamp;
    RefPtr<SharedBuffer> data;
};
    
class IconRecord : public RefCounted<IconRecord> {
    friend class PageURLRecord;
public:
    static PassRefPtr<IconRecord> create(const String& url)
    {
        return adoptRef(new IconRecord(url));
    }
    ~IconRecord();
    
    time_t getTimestamp() { return m_stamp; }
    void setTimestamp(time_t stamp) { m_stamp = stamp; }
        
    void setImageData(PassRefPtr<SharedBuffer> data);
    Image* image(const IntSize&);    
    
    String iconURL() { return m_iconURL; }

    void loadImageFromResource(const char*);
        
    ImageDataStatus imageDataStatus();
    
    const HashSet<String>& retainingPageURLs() { return m_retainingPageURLs; }
    
    IconSnapshot snapshot(bool forDeletion = false) const;

private:
    IconRecord(const String& url); 

    String m_iconURL;
    time_t m_stamp;
    RefPtr<Image> m_image;
    
    HashSet<String> m_retainingPageURLs;
        
    // This allows us to cache whether or not a SiteIcon has had its data set yet
    // This helps the IconDatabase know if it has to set the data on a new object or not,
    // and also to determine if the icon is missing data or if it just hasn't been brought
    // in from the DB yet
    bool m_dataSet;
    
    // FIXME - Right now WebCore::Image doesn't have a very good API for accessing multiple representations
    // Even the NSImage way of doing things that we do in WebKit isn't very clean...  once we come up with a 
    // better way of handling that, we'll likely have a map of size-to-images similar to below
    // typedef HashMap<IntSize, Image*> SizeImageMap;
    // SizeImageMap m_images;
};


} //namespace WebCore

#endif
