
#ifndef CachedFramePlatformData_h
#define CachedFramePlatformData_h

namespace WebCore {

// The purpose of this class is to give each platform a vessel to store platform data when a page
// goes into the Back/Forward page cache, and perform some action with that data when the page comes out.  
// Each platform should subclass this class as neccessary

class CachedFramePlatformData {
public:
    virtual ~CachedFramePlatformData() { }
    virtual void clear() { }
};

} // namespace WebCore

#endif // CachedFramePlatformData_h
