

namespace WebCore {

class KURL;
class String;

#if OS(WINDOWS)
void replaceNewlinesWithWindowsStyleNewlines(String&);
#endif
void replaceNBSPWithSpace(String&);

String urlToMarkup(const KURL&, const String&);

} // namespace WebCore
