

#include "config.h"
#include "FileChooser.h"

#include "FileSystem.h"
#include "LocalizedStrings.h"
#include "StringTruncator.h"

namespace WebCore {

String FileChooser::basenameForWidth(const Font& font, int width) const
{
    if (width <= 0)
        return String();

    String string;
    if (m_filenames.isEmpty())
        string = fileButtonNoFileSelectedLabel();
    else if (m_filenames.size() == 1)
        string = pathGetFileName(m_filenames[0]);
    else
        return StringTruncator::rightTruncate(String::number(m_filenames.size()) + " files", width, font, false);

    return StringTruncator::centerTruncate(string, width, font, false);
}

}
