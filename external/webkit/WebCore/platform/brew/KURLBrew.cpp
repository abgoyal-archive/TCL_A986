
#include "config.h"
#include "KURL.h"

#include "CString.h"

#include <AEEFile.h>

namespace WebCore {

String KURL::fileSystemPath() const
{
    // Access files relative to the current module directory in a case-sensitive manner.
    // As IWeb "file:" engine opens files in BREW's application-relative file namespace,
    // we follow the same policy here.
    return String(AEEFS_HOME_DIR) + path();
}

}
