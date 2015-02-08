

#ifndef ArchiveResource_h
#define ArchiveResource_h

#include "SubstituteResource.h"

#include "PlatformString.h"

namespace WebCore {

class ArchiveResource : public SubstituteResource {
public:
    static PassRefPtr<ArchiveResource> create(PassRefPtr<SharedBuffer>, const KURL&, const ResourceResponse&);
    static PassRefPtr<ArchiveResource> create(PassRefPtr<SharedBuffer>, const KURL&, const String& mimeType, const String& textEncoding, const String& frameName);
    static PassRefPtr<ArchiveResource> create(PassRefPtr<SharedBuffer>, const KURL&, const String& mimeType, const String& textEncoding, const String& frameName, const ResourceResponse&);
    
    const String& mimeType() const { return m_mimeType; }
    const String& textEncoding() const { return m_textEncoding; }
    const String& frameName() const { return m_frameName; }
    
    void ignoreWhenUnarchiving() { m_shouldIgnoreWhenUnarchiving = true; }
    bool shouldIgnoreWhenUnarchiving() const { return m_shouldIgnoreWhenUnarchiving; }

private:
    ArchiveResource(PassRefPtr<SharedBuffer>, const KURL&, const ResourceResponse&);
    ArchiveResource(PassRefPtr<SharedBuffer>, const KURL&, const String& mimeType, const String& textEncoding, const String& frameName);
    ArchiveResource(PassRefPtr<SharedBuffer>, const KURL&, const String& mimeType, const String& textEncoding, const String& frameName, const ResourceResponse&);
    
    String m_mimeType;
    String m_textEncoding;
    String m_frameName;
    
    bool m_shouldIgnoreWhenUnarchiving;
};

}

#endif // ArchiveResource_h
