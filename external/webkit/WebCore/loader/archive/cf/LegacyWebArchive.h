

#ifndef LegacyWebArchive_h
#define LegacyWebArchive_h

#include "Archive.h"

namespace WebCore {

class Frame;
class Node;
class Range;

class LegacyWebArchive : public Archive {
public:
    static PassRefPtr<LegacyWebArchive> create();
    static PassRefPtr<LegacyWebArchive> create(SharedBuffer*);
    static PassRefPtr<LegacyWebArchive> create(PassRefPtr<ArchiveResource> mainResource, Vector<PassRefPtr<ArchiveResource> >& subresources, Vector<PassRefPtr<LegacyWebArchive> >& subframeArchives);
    static PassRefPtr<LegacyWebArchive> create(Node*);
    static PassRefPtr<LegacyWebArchive> create(Frame*);
    static PassRefPtr<LegacyWebArchive> createFromSelection(Frame*);
    static PassRefPtr<LegacyWebArchive> create(Range*);

    RetainPtr<CFDataRef> rawDataRepresentation();

private:
    LegacyWebArchive() { }

    enum MainResourceStatus { Subresource, MainResource };

    static PassRefPtr<LegacyWebArchive> create(const String& markupString, Frame*, const Vector<Node*>& nodes);
    static PassRefPtr<ArchiveResource> createResource(CFDictionaryRef);
    static ResourceResponse createResourceResponseFromMacArchivedData(CFDataRef);
    static ResourceResponse createResourceResponseFromPropertyListData(CFDataRef, CFStringRef responseDataType);
    static RetainPtr<CFDataRef> createPropertyListRepresentation(const ResourceResponse&);
    static RetainPtr<CFDictionaryRef> createPropertyListRepresentation(Archive*);
    static RetainPtr<CFDictionaryRef> createPropertyListRepresentation(ArchiveResource*, MainResourceStatus);

    bool extract(CFDictionaryRef);
};

}

#endif // Archive
