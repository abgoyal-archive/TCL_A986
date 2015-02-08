

#include "config.h"
#include "KURL.h"

namespace WebCore {

String KURL::fileSystemPath() const
{
    return path();
}

}
