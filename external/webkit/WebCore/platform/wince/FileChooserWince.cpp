

#include "config.h"
#include "FileChooser.h"

#include "Chrome.h"
#include "ChromeClient.h"
#include "Document.h"
#include "Frame.h"
#include "Icon.h"
#include "LocalizedStrings.h"
#include "Page.h"
#include "StringTruncator.h"

namespace WebCore {

String pathGetFileName(const String& path);

String FileChooser::basenameForWidth(const Font& font, int width) const
{
    if (width <= 0)
        return String();

    String string;
    if (m_filenames.isEmpty())
        string = fileButtonNoFileSelectedLabel();
    else if (m_filenames.size() == 1) {
        String tmpFilename = m_filenames[0];
        string = pathGetFileName(tmpFilename);
    } else
        return StringTruncator::rightTruncate(String::number(m_filenames.size()) + " files", width, font, false);

    return StringTruncator::centerTruncate(string, width, font, false);
}

}
