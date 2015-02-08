

#include "config.h"
#include "FileChooser.h"

#include "CString.h"
#include "FileSystem.h"
#include "Icon.h"
#include "LocalizedStrings.h"
#include "StringTruncator.h"

#include <glib.h>
#include <gtk/gtk.h>

namespace WebCore {

static bool stringByAdoptingFileSystemRepresentation(gchar* systemFilename, String& result)
{
    if (!systemFilename)
        return false;

    result = filenameToString(systemFilename);
    g_free(systemFilename);

    return true;
}

String FileChooser::basenameForWidth(const Font& font, int width) const
{
    if (width <= 0)
        return String();

    String string = fileButtonNoFileSelectedLabel();

    if (m_filenames.size() == 1) {
        gchar* systemFilename = filenameFromString(m_filenames[0]);
        gchar* systemBasename = g_path_get_basename(systemFilename);
        g_free(systemFilename);
        stringByAdoptingFileSystemRepresentation(systemBasename, string);
    } else if (m_filenames.size() > 1)
        return StringTruncator::rightTruncate(multipleFileUploadText(m_filenames.size()), width, font, false);

    return StringTruncator::centerTruncate(string, width, font, false);
}
}
