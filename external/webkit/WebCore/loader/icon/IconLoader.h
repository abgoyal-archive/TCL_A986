

#ifndef IconLoader_h
#define IconLoader_h

#include "SubresourceLoaderClient.h"
#include <memory>
#include <wtf/Forward.h>
#include <wtf/Noncopyable.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class Frame;
class KURL;
class SharedBuffer;

class IconLoader : private SubresourceLoaderClient, public Noncopyable {
public:
    static std::auto_ptr<IconLoader> create(Frame*);
    ~IconLoader();
    
    void startLoading();
    void stopLoading();

private:
    IconLoader(Frame*);

    virtual void didReceiveResponse(SubresourceLoader*, const ResourceResponse&);
    virtual void didReceiveData(SubresourceLoader*, const char*, int);
    virtual void didFinishLoading(SubresourceLoader*);
    virtual void didFail(SubresourceLoader*, const ResourceError&);

    virtual void didReceiveAuthenticationChallenge(SubresourceLoader*, const AuthenticationChallenge&);
    
    void finishLoading(const KURL&, PassRefPtr<SharedBuffer> data);
    void clearLoadingState();

    Frame* m_frame;

    RefPtr<SubresourceLoader> m_resourceLoader;
    bool m_loadIsInProgress;
}; // class IconLoader

} // namespace WebCore

#endif
