

#ifndef HTMLInterchange_h
#define HTMLInterchange_h

namespace WebCore {

class String;
class Text;

#define AppleInterchangeNewline   "Apple-interchange-newline"
#define AppleConvertedSpace       "Apple-converted-space"
#define ApplePasteAsQuotation     "Apple-paste-as-quotation"
#define AppleStyleSpanClass       "Apple-style-span"
#define AppleTabSpanClass         "Apple-tab-span"

enum EAnnotateForInterchange { DoNotAnnotateForInterchange, AnnotateForInterchange };

String convertHTMLTextToInterchangeFormat(const String&, const Text*);

}

#endif
