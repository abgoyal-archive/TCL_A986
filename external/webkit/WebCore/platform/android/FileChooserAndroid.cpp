

#include "config.h"
#include "FileChooser.h"
#include "Font.h"

namespace WebCore {

String FileChooser::basenameForWidth(const Font& font, int width) const 
{
    if (!m_filenames.size())
        return String();
    // FIXME: This could be a lot faster, but assuming the data will not
    // often be much longer than the provided width, this may be fast enough.
    // If this does not need to be threadsafe, we can use crossThreadString().
    // See http://trac.webkit.org/changeset/49160.
    String output = m_filenames[0].threadsafeCopy();
    while (font.width(TextRun(output.impl())) > width && output.length() > 4)
        output = output.replace(0, 4, String("..."));
    return output;
}

} // namespace WebCore
