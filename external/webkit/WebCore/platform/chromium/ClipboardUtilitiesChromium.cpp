

#include "config.h"
#include "ClipboardUtilitiesChromium.h"

#include "KURL.h"
#include "PlatformString.h"

namespace WebCore {

#if OS(WINDOWS)
void replaceNewlinesWithWindowsStyleNewlines(String& str)
{
    static const UChar Newline = '\n';
    static const char* const WindowsNewline("\r\n");
    str.replace(Newline, WindowsNewline);
}
#endif

void replaceNBSPWithSpace(String& str)
{
    static const UChar NonBreakingSpaceCharacter = 0xA0;
    static const UChar SpaceCharacter = ' ';
    str.replace(NonBreakingSpaceCharacter, SpaceCharacter);
}

String urlToMarkup(const KURL& url, const String& title)
{
    String markup("<a href=\"");
    markup.append(url.string());
    markup.append("\">");
    // FIXME: HTML escape this, possibly by moving into the glue layer so we
    // can use net/base/escape.h.
    markup.append(title);
    markup.append("</a>");
    return markup;
}

} // namespace WebCore
