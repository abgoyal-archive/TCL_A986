

#ifndef IconFetcher_h
#define IconFetcher_h

#include <wtf/RefCounted.h>
#include <wtf/Forward.h>
#include <wtf/Vector.h>

#include "ResourceHandleClient.h"

namespace WebCore {

class Frame;
struct IconLinkEntry;
class ResourceHandle;
class SharedBuffer;

class IconFetcherClient {
public:
    virtual void finishedFetchingIcon(PassRefPtr<SharedBuffer> iconData) = 0;
    
    virtual ~IconFetcherClient() { }
};

class IconFetcher : public RefCounted<IconFetcher>, ResourceHandleClient {
public:
    static PassRefPtr<IconFetcher> create(Frame*, IconFetcherClient*);
    ~IconFetcher();
    
    void cancel();
    
private:
    IconFetcher(Frame*, IconFetcherClient*);
    void loadEntry();
    void loadFailed();
    
    PassRefPtr<SharedBuffer> createIcon();
    
    virtual void didReceiveResponse(ResourceHandle*, const ResourceResponse&);
    virtual void didReceiveData(ResourceHandle*, const char*, int, int lengthReceived);
    virtual void didFinishLoading(ResourceHandle*);
    virtual void didFail(ResourceHandle*, const ResourceError&);
    
    Frame* m_frame;
    IconFetcherClient* m_client;
    
    unsigned m_currentEntry;
    RefPtr<ResourceHandle> m_handle;
    Vector<IconLinkEntry> m_entries;
};
    
} // namespace WebCore

#endif // IconFetcher_h
