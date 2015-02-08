

#ifndef Archive_h
#define Archive_h

#include "ArchiveResource.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

class Archive : public RefCounted<Archive> {
public:    
    ArchiveResource* mainResource() { return m_mainResource.get(); }    
    const Vector<RefPtr<ArchiveResource> >& subresources() const { return m_subresources; }
    const Vector<RefPtr<Archive> >& subframeArchives() const { return m_subframeArchives; }

protected:
    // These methods are meant for subclasses for different archive types to add resources in to the archive,
    // and should not be exposed as archives should be immutable to clients
    void setMainResource(PassRefPtr<ArchiveResource> mainResource) { m_mainResource = mainResource; }
    void addSubresource(PassRefPtr<ArchiveResource> subResource) { m_subresources.append(subResource); }
    void addSubframeArchive(PassRefPtr<Archive> subframeArchive) { m_subframeArchives.append(subframeArchive); }
    
private:
    RefPtr<ArchiveResource> m_mainResource;
    Vector<RefPtr<ArchiveResource> > m_subresources;
    Vector<RefPtr<Archive> > m_subframeArchives;
};

}

#endif // Archive
