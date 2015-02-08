

#include "config.h"
#include "KURL.h"

#include "CString.h"
#include "FileSystem.h"

#include <glib.h>

namespace WebCore {

String KURL::fileSystemPath() const
{
    gchar* filename = g_filename_from_uri(m_string.utf8().data(), 0, 0);
    if (!filename)
        return String();

    String path = filenameToString(filename);
    g_free(filename);
    return path;
}

} // namespace WebCore
