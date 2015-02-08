

#include "config.h"
#include "OwnPtrBrew.h"

#include <AEEBitmap.h>
#include <AEEFile.h>
#include <AEEStdLib.h>

namespace WTF {

template <> void freeOwnedPtrBrew<IFileMgr>(IFileMgr* ptr)
{
    if (ptr)
        IFILEMGR_Release(ptr);
}

template <> void freeOwnedPtrBrew<IFile>(IFile* ptr)
{
    if (ptr)
        IFILE_Release(ptr);
}

template <> void freeOwnedPtrBrew<IBitmap>(IBitmap* ptr)
{
    if (ptr)
        IBitmap_Release(ptr);
}

template <typename T> void freeOwnedPtrBrew(T* ptr)
{
    FREEIF(ptr);
}

} // namespace WTF
