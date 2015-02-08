

#include "config.h"
#include "SharedBuffer.h"

#include "CString.h"
#include "FileSystem.h"

#include <glib.h>


namespace WebCore {

PassRefPtr<SharedBuffer> SharedBuffer::createWithContentsOfFile(const String& filePath)
{
    if (filePath.isEmpty())
        return 0;

    gchar* filename = filenameFromString(filePath);
    gchar* contents;
    gsize size;
    GError* error = 0;
    if (!g_file_get_contents(filename, &contents, &size, &error)) {
        LOG_ERROR("Failed to fully read contents of file %s - %s", filenameForDisplay(filePath).utf8().data(), error->message);
        g_error_free(error);
        g_free(filename);
        return 0;
    }

    RefPtr<SharedBuffer> result = SharedBuffer::create(contents, size);
    g_free(filename);
    g_free(contents);

    return result.release();
}

} // namespace WebCore
