

#include "config.h"
#include "ArchiveFactory.h"

#include "MIMETypeRegistry.h"
#include "PlatformString.h"

#if PLATFORM(CF) && !PLATFORM(QT)
#include "LegacyWebArchive.h"
#endif

#include <wtf/HashMap.h>
#include <wtf/HashSet.h>
#include <wtf/StdLibExtras.h>

namespace WebCore {

typedef PassRefPtr<Archive> RawDataCreationFunction(SharedBuffer*);
typedef HashMap<String, RawDataCreationFunction*, CaseFoldingHash> ArchiveMIMETypesMap;

// The create functions in the archive classes return PassRefPtr to concrete subclasses
// of Archive. This adaptor makes the functions have a uniform return type.
template <typename ArchiveClass> static PassRefPtr<Archive> archiveFactoryCreate(SharedBuffer* buffer)
{
    return ArchiveClass::create(buffer);
}

static ArchiveMIMETypesMap& archiveMIMETypes()
{
    DEFINE_STATIC_LOCAL(ArchiveMIMETypesMap, mimeTypes, ());
    static bool initialized = false;

    if (initialized)
        return mimeTypes;

#if PLATFORM(CF) && !PLATFORM(QT)
    mimeTypes.set("application/x-webarchive", archiveFactoryCreate<LegacyWebArchive>);
#endif

    initialized = true;
    return mimeTypes;
}

bool ArchiveFactory::isArchiveMimeType(const String& mimeType)
{
    return archiveMIMETypes().contains(mimeType);
}

PassRefPtr<Archive> ArchiveFactory::create(SharedBuffer* data, const String& mimeType)
{
    RawDataCreationFunction* function = archiveMIMETypes().get(mimeType);
    return function ? function(data) : 0;
}

void ArchiveFactory::registerKnownArchiveMIMETypes()
{
    HashSet<String>& mimeTypes = MIMETypeRegistry::getSupportedNonImageMIMETypes();
    ArchiveMIMETypesMap::iterator i = archiveMIMETypes().begin();
    ArchiveMIMETypesMap::iterator end = archiveMIMETypes().end();
    
    for (; i != end; ++i)
        mimeTypes.add(i->first);
}

}
