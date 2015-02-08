

#ifndef ArchiveFactory_h
#define ArchiveFactory_h

#include "Archive.h"

#include <wtf/PassRefPtr.h>

namespace WebCore {

class SharedBuffer;
class String;

class ArchiveFactory {
public:
    static bool isArchiveMimeType(const String&);
    static PassRefPtr<Archive> create(SharedBuffer* data, const String& mimeType);
    static void registerKnownArchiveMIMETypes();
};

}

#endif // ArchiveFactory_h
