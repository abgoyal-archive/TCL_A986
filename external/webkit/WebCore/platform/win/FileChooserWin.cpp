

#include "config.h"
#include "FileChooser.h"

#include "LocalizedStrings.h"
#include "StringTruncator.h"
#include <shlwapi.h>
#include <tchar.h>
#include <windows.h>

namespace WebCore {

String FileChooser::basenameForWidth(const Font& font, int width) const
{
    if (width <= 0)
        return String();

    String string;
    if (m_filenames.isEmpty())
        string = fileButtonNoFileSelectedLabel();
    else if (m_filenames.size() == 1) {
        String tmpFilename = m_filenames[0];
        LPTSTR basename = PathFindFileName(tmpFilename.charactersWithNullTermination());
        string = String(basename);
    } else
        return StringTruncator::rightTruncate(String::number(m_filenames.size()) + " files", width, font, false);

    return StringTruncator::centerTruncate(string, width, font, false);
}

} // namespace WebCore
