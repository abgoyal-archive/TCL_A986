

#ifndef ArchiveResourceCollection_h
#define ArchiveResourceCollection_h

#include "Archive.h"
#include "ArchiveResource.h"
#include "KURL.h"
#include "PlatformString.h"

#include <wtf/HashMap.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class ArchiveResourceCollection : public Noncopyable {
public:
    ArchiveResourceCollection();

    void addResource(PassRefPtr<ArchiveResource>);
    void addAllResources(Archive*);
    
    ArchiveResource* archiveResourceForURL(const KURL&);
    PassRefPtr<Archive> popSubframeArchive(const String& frameName);
    
private:    
    HashMap<String, RefPtr<ArchiveResource> > m_subresources;
    HashMap<String, RefPtr<Archive> > m_subframes;
};

}

#endif
